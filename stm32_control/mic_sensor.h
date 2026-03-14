/**
 * Digital Microphone Driver for Acoustic Tap Analysis
 * ADC-based capture with DMA circular buffer and CMSIS-DSP FFT.
 *
 * Uses ADC3 on an available pin from the 40-pin header for analog
 * microphone input. Configured for fast sampling (~8kHz) with
 * DMA-driven circular buffer for tap capture and spectral analysis.
 */

#ifndef MIC_SENSOR_H
#define MIC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================== */
/*                       MICROPHONE CONFIGURATION                              */
/* ========================================================================== */

/* ADC3 analog microphone input pin (PF3 on 40-pin header, ADC3_IN5) */
#define MIC_ADC_INSTANCE          ADC3
#define MIC_ADC_CHANNEL           ADC_CHANNEL_5
#define MIC_ADC_PIN               GPIO_PIN_3
#define MIC_ADC_PORT              GPIOF
#define MIC_ADC_IRQn              ADC3_IRQn
#define MIC_ADC_IRQ_PRIORITY      7U

/* DMA for ADC3 (BDMA on D3 domain for ADC3 on STM32H725) */
#define MIC_DMA_INSTANCE          BDMA_Channel0
#define MIC_DMA_REQUEST           BDMA_REQUEST_ADC3
#define MIC_DMA_IRQn              BDMA_Channel0_IRQn
#define MIC_DMA_IRQ_PRIORITY      7U

/* Sampling parameters */
#define MIC_SAMPLE_RATE_HZ        8000U
#define MIC_BUFFER_SIZE           1600U    /* 200ms at 8kHz */
#define MIC_TRIGGER_THRESHOLD     2200U    /* ADC counts above baseline for tap detection */
#define MIC_BASELINE_ADC          2048U    /* Mid-scale for 12-bit ADC */

/* FFT parameters */
#define MIC_FFT_SIZE              1024U
#define MIC_FFT_HALF              (MIC_FFT_SIZE / 2U)

/* Timer for ADC trigger (TIM6 - basic timer, APB1 domain) */
#define MIC_TIM_INSTANCE          TIM6
#define MIC_TIM_PRESCALER         274U     /* 275MHz / (274+1) = 1MHz tick */
#define MIC_TIM_PERIOD            124U     /* 1MHz / (124+1) = 8kHz */

/* ========================================================================== */
/*                       DATA STRUCTURES                                       */
/* ========================================================================== */

typedef struct {
    uint16_t samples[MIC_BUFFER_SIZE];
    uint16_t sample_count;
    uint32_t sample_rate;
    bool     capture_complete;
    float    dominant_freq_hz;
    float    spectral_centroid_hz;
    float    decay_ratio;
} MicCapture_t;

/* Capture state machine */
typedef enum {
    MIC_STATE_IDLE,
    MIC_STATE_WAITING_TRIGGER,
    MIC_STATE_CAPTURING,
    MIC_STATE_COMPLETE,
    MIC_STATE_ERROR,
} MicState_t;

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

HAL_StatusTypeDef initMicrophone(void);
void              triggerTapCapture(void);
uint16_t         *getTapBuffer(uint16_t *sample_count);
bool              isCaptureComplete(void);
void              computeBasicFFT(MicCapture_t *capture);

/* DMA callbacks (called from IRQ) */
void              micDmaHalfCpltCallback(void);
void              micDmaCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* MIC_SENSOR_H */
