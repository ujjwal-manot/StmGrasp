/**
 * Digital Microphone Driver Implementation
 * ADC3-based analog microphone capture with DMA circular buffer.
 *
 * Uses TIM6 as ADC trigger at 8kHz sample rate. DMA transfers samples
 * into a circular buffer. Tap detection uses a threshold crossing to
 * start capture of 1600 samples (200ms window).
 *
 * Spectral analysis uses ARM CMSIS-DSP library (arm_rfft_fast_f32)
 * which is highly optimized for Cortex-M7 with hardware FPU.
 *
 * ADC3 on STM32H725 is in the D3 domain and uses BDMA (not DMA1/DMA2).
 * The DMA buffer must be placed in SRAM4 (D3 domain accessible).
 *
 * Reference: STM32H725 Reference Manual RM0468.
 */

#include "mic_sensor.h"
#include "arm_math.h"

/* ========================================================================== */
/*  Static variables                                                           */
/* ========================================================================== */

static ADC_HandleTypeDef  s_hadc3;
static DMA_HandleTypeDef  s_hdmaBdma;
static TIM_HandleTypeDef  s_htim6;

static volatile MicState_t s_micState = MIC_STATE_IDLE;

/* DMA buffer in SRAM4 (D3 domain) for ADC3/BDMA compatibility.
 * On STM32H725, ADC3 uses BDMA which can only access SRAM4.
 * Place buffer in .sram4 section via linker attribute. */
static uint16_t s_dmaBuffer[MIC_BUFFER_SIZE] __attribute__((section(".sram4")));

/* Capture buffer (copied from DMA buffer after capture completes) */
static MicCapture_t s_capture;

/* FFT working buffers (in normal SRAM, accessed by CPU only) */
static float s_fftInput[MIC_FFT_SIZE];
static float s_fftOutput[MIC_FFT_SIZE];
static float s_fftMagnitude[MIC_FFT_HALF];

/* CMSIS-DSP FFT instance */
static arm_rfft_fast_instance_f32 s_fftInstance;

/* Sample counter for trigger-based capture */
static volatile uint16_t s_captureIndex = 0;
static volatile bool     s_triggerArmed = false;

/* ========================================================================== */
/*  ADC3 + BDMA + TIM6 initialization                                         */
/* ========================================================================== */

/**
 * Initialize ADC3 with DMA for analog microphone capture.
 * TIM6 provides the 8kHz sample clock as ADC trigger source.
 * BDMA Channel 0 transfers ADC data to SRAM4 buffer.
 */
HAL_StatusTypeDef initMicrophone(void)
{
    HAL_StatusTypeDef ret;
    GPIO_InitTypeDef gpio = {0};

    /* ---- Enable clocks ---- */
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* ---- Configure analog input pin: PF3 (ADC3_IN5) ---- */
    gpio.Pin  = MIC_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MIC_ADC_PORT, &gpio);

    /* ---- Initialize CMSIS-DSP FFT ---- */
    arm_rfft_fast_init_f32(&s_fftInstance, MIC_FFT_SIZE);

    /* ---- Configure TIM6 as ADC trigger at 8kHz ---- */
    s_htim6.Instance               = MIC_TIM_INSTANCE;
    s_htim6.Init.Prescaler         = MIC_TIM_PRESCALER;
    s_htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim6.Init.Period            = MIC_TIM_PERIOD;
    s_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    ret = HAL_TIM_Base_Init(&s_htim6);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Configure TIM6 TRGO as update event (for ADC trigger) */
    TIM_MasterConfigTypeDef masterCfg = {0};
    masterCfg.MasterOutputTrigger  = TIM_TRGO_UPDATE;
    masterCfg.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    ret = HAL_TIMEx_MasterConfigSynchronization(&s_htim6, &masterCfg);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* ---- Configure BDMA Channel 0 for ADC3 ---- */
    s_hdmaBdma.Instance                 = MIC_DMA_INSTANCE;
    s_hdmaBdma.Init.Request             = MIC_DMA_REQUEST;
    s_hdmaBdma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdmaBdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdmaBdma.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdmaBdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdmaBdma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdmaBdma.Init.Mode                = DMA_CIRCULAR;
    s_hdmaBdma.Init.Priority            = DMA_PRIORITY_HIGH;

    ret = HAL_DMA_Init(&s_hdmaBdma);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Link DMA to ADC3 */
    __HAL_LINKDMA(&s_hadc3, DMA_Handle, s_hdmaBdma);

    /* Enable BDMA interrupt */
    HAL_NVIC_SetPriority(MIC_DMA_IRQn, MIC_DMA_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(MIC_DMA_IRQn);

    /* ---- Configure ADC3 ---- */
    s_hadc3.Instance                      = MIC_ADC_INSTANCE;
    s_hadc3.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV4;
    s_hadc3.Init.Resolution               = ADC_RESOLUTION_12B;
    s_hadc3.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    s_hadc3.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    s_hadc3.Init.LowPowerAutoWait         = DISABLE;
    s_hadc3.Init.ContinuousConvMode       = DISABLE;
    s_hadc3.Init.NbrOfConversion          = 1;
    s_hadc3.Init.DiscontinuousConvMode    = DISABLE;
    s_hadc3.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T6_TRGO;
    s_hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc3.Init.DMAContinuousRequests    = ENABLE;
    s_hadc3.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;

    ret = HAL_ADC_Init(&s_hadc3);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Configure ADC3 channel */
    ADC_ChannelConfTypeDef chCfg = {0};
    chCfg.Channel      = MIC_ADC_CHANNEL;
    chCfg.Rank         = ADC_REGULAR_RANK_1;
    chCfg.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    chCfg.SingleDiff   = ADC_SINGLE_ENDED;
    chCfg.OffsetNumber = ADC_OFFSET_NONE;

    ret = HAL_ADC_ConfigChannel(&s_hadc3, &chCfg);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Run ADC calibration */
    ret = HAL_ADCEx_Calibration_Start(&s_hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Clear capture state */
    memset(&s_capture, 0, sizeof(MicCapture_t));
    s_capture.sample_rate = MIC_SAMPLE_RATE_HZ;
    s_captureIndex = 0;
    s_triggerArmed = false;
    s_micState = MIC_STATE_IDLE;

    /* Start ADC with DMA in circular mode (always sampling) */
    ret = HAL_ADC_Start_DMA(&s_hadc3, (uint32_t *)s_dmaBuffer, MIC_BUFFER_SIZE);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    /* Start TIM6 to begin triggering ADC conversions */
    ret = HAL_TIM_Base_Start(&s_htim6);
    if (ret != HAL_OK) {
        s_micState = MIC_STATE_ERROR;
        return ret;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  BDMA IRQ handler                                                           */
/* ========================================================================== */

/**
 * BDMA Channel 0 interrupt handler. Must be called from the
 * BDMA_Channel0_IRQHandler in the startup/interrupt vector.
 */
void BDMA_Channel0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdmaBdma);
}

/* ========================================================================== */
/*  DMA callbacks                                                              */
/* ========================================================================== */

/**
 * DMA half-complete callback: first half of circular buffer is filled.
 * Used for trigger detection when waiting for a tap.
 */
void micDmaHalfCpltCallback(void)
{
    if (s_micState == MIC_STATE_WAITING_TRIGGER) {
        /* Scan the first half of the buffer for threshold crossing */
        for (uint16_t i = 0; i < MIC_BUFFER_SIZE / 2; i++) {
            int16_t deviation = (int16_t)s_dmaBuffer[i] - (int16_t)MIC_BASELINE_ADC;
            if (deviation < 0) deviation = -deviation;

            if ((uint16_t)deviation > (MIC_TRIGGER_THRESHOLD - MIC_BASELINE_ADC)) {
                /* Tap detected: switch to capturing mode */
                s_micState = MIC_STATE_CAPTURING;
                s_captureIndex = 0;

                /* Copy remaining samples from this half-buffer */
                uint16_t remaining = (MIC_BUFFER_SIZE / 2) - i;
                memcpy(s_capture.samples, &s_dmaBuffer[i], remaining * sizeof(uint16_t));
                s_captureIndex = remaining;
                return;
            }
        }
    } else if (s_micState == MIC_STATE_CAPTURING) {
        /* Continue filling capture buffer from first half of DMA buffer */
        uint16_t space = MIC_BUFFER_SIZE - s_captureIndex;
        uint16_t to_copy = MIC_BUFFER_SIZE / 2;
        if (to_copy > space) to_copy = space;

        if (to_copy > 0) {
            memcpy(&s_capture.samples[s_captureIndex], s_dmaBuffer, to_copy * sizeof(uint16_t));
            s_captureIndex += to_copy;
        }

        if (s_captureIndex >= MIC_BUFFER_SIZE) {
            s_capture.sample_count = MIC_BUFFER_SIZE;
            s_capture.capture_complete = true;
            s_micState = MIC_STATE_COMPLETE;
        }
    }
}

/**
 * DMA full-complete callback: second half of circular buffer is filled.
 * Continues capture or checks for trigger in the second half.
 */
void micDmaCpltCallback(void)
{
    if (s_micState == MIC_STATE_WAITING_TRIGGER) {
        /* Scan the second half for threshold crossing */
        uint16_t half_offset = MIC_BUFFER_SIZE / 2;
        for (uint16_t i = half_offset; i < MIC_BUFFER_SIZE; i++) {
            int16_t deviation = (int16_t)s_dmaBuffer[i] - (int16_t)MIC_BASELINE_ADC;
            if (deviation < 0) deviation = -deviation;

            if ((uint16_t)deviation > (MIC_TRIGGER_THRESHOLD - MIC_BASELINE_ADC)) {
                s_micState = MIC_STATE_CAPTURING;
                s_captureIndex = 0;

                uint16_t remaining = MIC_BUFFER_SIZE - i;
                memcpy(s_capture.samples, &s_dmaBuffer[i], remaining * sizeof(uint16_t));
                s_captureIndex = remaining;
                return;
            }
        }
    } else if (s_micState == MIC_STATE_CAPTURING) {
        /* Continue filling capture buffer from second half of DMA buffer */
        uint16_t space = MIC_BUFFER_SIZE - s_captureIndex;
        uint16_t to_copy = MIC_BUFFER_SIZE / 2;
        if (to_copy > space) to_copy = space;

        if (to_copy > 0) {
            uint16_t half_offset = MIC_BUFFER_SIZE / 2;
            memcpy(&s_capture.samples[s_captureIndex], &s_dmaBuffer[half_offset],
                   to_copy * sizeof(uint16_t));
            s_captureIndex += to_copy;
        }

        if (s_captureIndex >= MIC_BUFFER_SIZE) {
            s_capture.sample_count = MIC_BUFFER_SIZE;
            s_capture.capture_complete = true;
            s_micState = MIC_STATE_COMPLETE;
        }
    }
}

/**
 * HAL DMA callbacks wired to our handlers.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == MIC_ADC_INSTANCE) {
        micDmaHalfCpltCallback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == MIC_ADC_INSTANCE) {
        micDmaCpltCallback();
    }
}

/* ========================================================================== */
/*  Trigger tap capture                                                        */
/* ========================================================================== */

/**
 * Arm the microphone for tap capture.
 * Sets a trigger flag; the DMA callbacks will detect threshold crossing
 * (tap impact) and then capture MIC_BUFFER_SIZE samples.
 */
void triggerTapCapture(void)
{
    /* Reset capture state */
    s_capture.capture_complete = false;
    s_capture.sample_count = 0;
    s_capture.dominant_freq_hz = 0.0f;
    s_capture.spectral_centroid_hz = 0.0f;
    s_capture.decay_ratio = 0.0f;
    s_captureIndex = 0;
    s_triggerArmed = true;
    s_micState = MIC_STATE_WAITING_TRIGGER;
}

/* ========================================================================== */
/*  Buffer access                                                              */
/* ========================================================================== */

/**
 * Return pointer to completed capture buffer and sample count.
 * Returns NULL if capture is not complete.
 */
uint16_t *getTapBuffer(uint16_t *sample_count)
{
    if (!s_capture.capture_complete) {
        if (sample_count != NULL) {
            *sample_count = 0;
        }
        return NULL;
    }

    if (sample_count != NULL) {
        *sample_count = s_capture.sample_count;
    }
    return s_capture.samples;
}

/**
 * Check if DMA capture is complete.
 */
bool isCaptureComplete(void)
{
    return s_capture.capture_complete;
}

/* ========================================================================== */
/*  FFT spectral analysis                                                      */
/* ========================================================================== */

/**
 * Compute basic FFT spectral analysis on the captured audio data.
 * Uses ARM CMSIS-DSP arm_rfft_fast_f32 for 1024-point real FFT.
 *
 * Computes:
 *   - Dominant frequency: bin with maximum magnitude
 *   - Spectral centroid: weighted average of frequency bins
 *   - Decay ratio: ratio of energy in last quarter vs first quarter
 *
 * The STM32H725 Cortex-M7 FPU makes this very fast (~0.5ms for 1024-pt FFT).
 */
void computeBasicFFT(MicCapture_t *capture)
{
    if (!capture->capture_complete || capture->sample_count < MIC_FFT_SIZE) {
        capture->dominant_freq_hz = 0.0f;
        capture->spectral_centroid_hz = 0.0f;
        capture->decay_ratio = 0.0f;
        return;
    }

    /* Convert uint16_t ADC samples to float, removing DC offset */
    for (uint16_t i = 0; i < MIC_FFT_SIZE; i++) {
        s_fftInput[i] = (float)capture->samples[i] - (float)MIC_BASELINE_ADC;
    }

    /* Apply Hanning window to reduce spectral leakage */
    for (uint16_t i = 0; i < MIC_FFT_SIZE; i++) {
        float window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float)i / (float)(MIC_FFT_SIZE - 1)));
        s_fftInput[i] *= window;
    }

    /* Compute 1024-point real FFT */
    arm_rfft_fast_f32(&s_fftInstance, s_fftInput, s_fftOutput, 0);

    /* Compute magnitude spectrum (first half only due to symmetry).
     * Output format: [Re[0], Re[N/2], Re[1], Im[1], Re[2], Im[2], ...] */
    s_fftMagnitude[0] = fabsf(s_fftOutput[0]); /* DC component */

    for (uint16_t i = 1; i < MIC_FFT_HALF; i++) {
        float real = s_fftOutput[2 * i];
        float imag = s_fftOutput[2 * i + 1];
        float mag;
        arm_sqrt_f32(real * real + imag * imag, &mag);
        s_fftMagnitude[i] = mag;
    }

    /* Find dominant frequency (skip DC bin at index 0) */
    float max_mag = 0.0f;
    uint16_t max_bin = 1;

    for (uint16_t i = 1; i < MIC_FFT_HALF; i++) {
        if (s_fftMagnitude[i] > max_mag) {
            max_mag = s_fftMagnitude[i];
            max_bin = i;
        }
    }

    /* Frequency resolution: sample_rate / FFT_size */
    float freq_resolution = (float)MIC_SAMPLE_RATE_HZ / (float)MIC_FFT_SIZE;
    capture->dominant_freq_hz = (float)max_bin * freq_resolution;

    /* Compute spectral centroid: sum(f * |X(f)|) / sum(|X(f)|) */
    float weighted_sum = 0.0f;
    float magnitude_sum = 0.0f;

    for (uint16_t i = 1; i < MIC_FFT_HALF; i++) {
        float freq = (float)i * freq_resolution;
        weighted_sum += freq * s_fftMagnitude[i];
        magnitude_sum += s_fftMagnitude[i];
    }

    if (magnitude_sum > 0.001f) {
        capture->spectral_centroid_hz = weighted_sum / magnitude_sum;
    } else {
        capture->spectral_centroid_hz = 0.0f;
    }

    /* Compute decay ratio: energy in last quarter vs first quarter of time signal.
     * Low decay ratio = fast decay = hard material (metal, glass).
     * High decay ratio = slow decay = soft material (wood, plastic). */
    float energy_first = 0.0f;
    float energy_last = 0.0f;
    uint16_t quarter = capture->sample_count / 4;

    for (uint16_t i = 0; i < quarter; i++) {
        float val = (float)capture->samples[i] - (float)MIC_BASELINE_ADC;
        energy_first += val * val;
    }

    for (uint16_t i = capture->sample_count - quarter; i < capture->sample_count; i++) {
        float val = (float)capture->samples[i] - (float)MIC_BASELINE_ADC;
        energy_last += val * val;
    }

    if (energy_first > 0.001f) {
        capture->decay_ratio = energy_last / energy_first;
    } else {
        capture->decay_ratio = 0.0f;
    }
}
