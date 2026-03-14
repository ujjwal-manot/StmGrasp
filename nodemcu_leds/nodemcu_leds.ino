/*
 * nodemcu_leds.ino
 * WS2812B LED "Nervous System" controller for robotic gripper
 *
 * Hardware: NodeMCU ESP8266, 20x WS2812B strip on GPIO2 (D4)
 * Comms:    UART 9600 baud from ESP32 brain
 * Protocol: [0xBB][CMD][P1][P2][P3]
 *
 * (c) 2026 Nephora / StmGrasp project
 */

#include <FastLED.h>

/* ── Hardware config ───────────────────────────────────────────────── */
#define LED_PIN       2          // GPIO2 = D4 on NodeMCU
#define NUM_LEDS      20
#define COLOR_ORDER   GRB
#define CHIPSET       WS2812B
#define MASTER_BRIGHT 150

/* ── Timing ────────────────────────────────────────────────────────── */
#define FRAME_INTERVAL_MS  16   // ~60 FPS
#define UART_TIMEOUT_MS    50   // max time to accumulate a packet

/* ── UART protocol ─────────────────────────────────────────────────── */
#define SYNC_BYTE     0xBB
#define PACKET_LEN    5

#define CMD_SOLID       0x01
#define CMD_ANIMATE     0x02
#define CMD_PIXEL       0x03
#define CMD_MATERIAL    0x04
#define CMD_HEARTBEAT   0x05
#define CMD_FORCE_MAP   0x06

/* ── Animation IDs ─────────────────────────────────────────────────── */
enum AnimID : uint8_t {
  ANIM_HEARTBEAT      = 0,
  ANIM_SEARCH_SWEEP   = 1,
  ANIM_DETECTED_FLASH = 2,
  ANIM_RAINBOW        = 3,
  ANIM_PLANNING_PULSE = 4,
  ANIM_APPROACH_RAMP  = 5,
  ANIM_GRIP_FILL      = 6,
  ANIM_HOLD_STEADY    = 7,
  ANIM_RELEASE_FADE   = 8,
  ANIM_ERROR_STROBE   = 9,
  ANIM_TAP_FLASH      = 10,
  ANIM_NONE           = 0xFF
};

/* ── Material palette ──────────────────────────────────────────────── */
static const CRGB materialColors[] = {
  CRGB(40,  80, 220),   // 0 Metal   - blue
  CRGB(30, 200,  50),   // 1 Skin    - green
  CRGB(220, 30,  20),   // 2 Plastic - red
  CRGB(220, 200, 20),   // 3 Wood    - yellow
  CRGB(20, 200, 200),   // 4 Glass   - cyan
  CRGB(230, 120, 10),   // 5 Cardboard - orange
};
#define NUM_MATERIALS 6

/* ── Global state ──────────────────────────────────────────────────── */
CRGB leds[NUM_LEDS];
CRGB crossfadeBuf[NUM_LEDS];   // snapshot for crossfade

// Current animation
volatile AnimID  curAnim      = ANIM_NONE;
volatile uint8_t animSpeed    = 5;          // 1-10
volatile bool    animChanged  = false;

// Solid-color override
CRGB solidColor = CRGB::Black;
bool solidMode  = false;

// Heartbeat BPM (fixed-point: real BPM * 10)
uint16_t heartbeatBpmX10 = 600;   // 60.0 BPM default

// Material
uint8_t  materialId   = 0;
uint8_t  materialConf = 0;
CRGB     materialCol  = materialColors[0];

// Force heatmap (3 FSR zones)
uint8_t  forceLevel[3] = {0, 0, 0};

// Approach fill level (0..NUM_LEDS)
uint8_t  approachFill = 0;

// Crossfade
uint8_t  crossfadeAlpha = 255;     // 255 = fully new, 0 = fully old
bool     crossfading    = false;
uint32_t crossfadeStart = 0;
#define  CROSSFADE_MS   120

// Detected-flash sub-state
uint8_t  detFlashCount   = 0;
uint32_t detFlashStamp   = 0;
bool     detFlashOn      = false;
bool     detFlashDone    = false;

// Tap-flash sub-state
uint32_t tapFlashStamp   = 0;
bool     tapFlashOn      = false;
bool     tapFlashDone    = false;

// Release-fade sub-state
uint32_t releaseFadeStart = 0;
CRGB     releaseFadeSnap[NUM_LEDS];
bool     releaseFadeInit  = false;

// Frame clock
uint32_t lastFrameMs = 0;
uint32_t animStartMs = 0;   // timestamp when current animation started

/* ── UART receive buffer ───────────────────────────────────────────── */
uint8_t  rxBuf[PACKET_LEN];
uint8_t  rxIdx        = 0;
uint32_t rxLastByte   = 0;

/* ══════════════════════════════════════════════════════════════════════
 *  HELPERS
 * ══════════════════════════════════════════════════════════════════════ */

static inline void setAllLEDs(CRGB c) {
  fill_solid(leds, NUM_LEDS, c);
}

/* Scale a CRGB by 0-255 brightness without floats */
static inline CRGB scaleBrightness(CRGB c, uint8_t b) {
  return CRGB(scale8(c.r, b), scale8(c.g, b), scale8(c.b, b));
}

/* Blend two CRGB arrays element-wise: out = A*(255-t)/255 + B*t/255 */
static void blendArrays(CRGB *dst, const CRGB *a, const CRGB *b,
                         uint8_t t, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    dst[i] = blend(a[i], b[i], t);
  }
}

/* Integer-only HSV-to-RGB (hue 0-255, sat 0-255, val 0-255)
   Just forwards to FastLED's optimised version. */
static inline CRGB hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  CHSV hsv(h, s, v);
  CRGB rgb;
  hsv2rgb_rainbow(hsv, rgb);
  return rgb;
}

/* Smooth sine-ish pulse using FastLED's sin8 (0-255 result).
   phase is 0-255 representing 0-2pi. */
static inline uint8_t sinPulse(uint8_t phase) {
  uint8_t s = sin8(phase);                // 0-255
  return scale8(s, s);                     // squared for sharper pulse
}

/* Map 0-255 linearly to 0-maxVal */
static inline uint8_t map8(uint8_t x, uint8_t maxVal) {
  return (uint16_t)x * maxVal / 255;
}

/* ══════════════════════════════════════════════════════════════════════
 *  ANIMATION FUNCTIONS
 *  Each takes elapsed milliseconds since animation start.
 *  They write directly into leds[].
 * ══════════════════════════════════════════════════════════════════════ */

/* 0 ── Heartbeat ──────────────────────────────────────────────────── */
static void animHeartbeat(uint32_t elapsed) {
  /*  Traveling sine pulse along strip.
      Period derived from BPM:  period_ms = 60000 * 10 / bpmX10        */
  uint32_t periodMs = (uint32_t)600000UL / (uint32_t)heartbeatBpmX10;
  if (periodMs < 200) periodMs = 200;

  // phase accumulator: 0-255 maps to one full period
  uint8_t timePhase = (uint8_t)((elapsed % periodMs) * 256UL / periodMs);

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    // spatial offset: each LED is offset by 256/NUM_LEDS
    uint8_t spatialOff = (uint8_t)((uint16_t)i * 256 / NUM_LEDS);
    uint8_t phase = timePhase - spatialOff;  // wraps naturally
    uint8_t brightness = sinPulse(phase);
    // Deep blue: keep R=0, G=0, B=brightness
    leds[i] = CRGB(0, 0, brightness);
  }
}

/* 1 ── Search Sweep ───────────────────────────────────────────────── */
static void animSearchSweep(uint32_t elapsed) {
  // Background: dim blue
  fill_solid(leds, NUM_LEDS, CRGB(5, 5, 15));

  // 1 full sweep (there-and-back) per second = 2*NUM_LEDS positions/s
  // Forward + backward = 2*(NUM_LEDS-1) steps per cycle
  uint16_t cycleSteps = 2 * (NUM_LEDS - 1);    // 38
  uint16_t stepDurMs  = 1000 / cycleSteps;      // ~26ms per step
  if (stepDurMs == 0) stepDurMs = 1;
  uint16_t step = (uint16_t)((elapsed / stepDurMs) % cycleSteps);

  int16_t pos;
  if (step < NUM_LEDS) {
    pos = step;
  } else {
    pos = cycleSteps - step;
  }

  // Draw head + 3-LED trail
  for (int8_t t = 0; t < 4; t++) {
    int16_t idx = pos - t;
    if (idx < 0 || idx >= NUM_LEDS) continue;
    uint8_t bright = 255 >> t;   // 255, 127, 63, 31
    leds[idx] = CRGB(bright, bright, bright);
  }
}

/* 2 ── Detected Flash ─────────────────────────────────────────────── */
static void animDetectedFlash(uint32_t elapsed) {
  /* 3 flashes: 100ms on, 100ms off each → 600ms total, then solid dim */

  if (!detFlashDone) {
    uint8_t flashPhase = (uint8_t)(elapsed / 100);   // 0,1,2,3,4,5,...
    if (flashPhase < 6) {
      bool on = (flashPhase & 1) == 0;  // even = on
      if (on) {
        fill_solid(leds, NUM_LEDS, CRGB(255, 200, 0));
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
    } else {
      detFlashDone = true;
    }
  }

  if (detFlashDone) {
    // Settle to dim yellow
    fill_solid(leds, NUM_LEDS, CRGB(60, 50, 0));
  }
}

/* 3 ── Rainbow Analyze ────────────────────────────────────────────── */
static void animRainbow(uint32_t elapsed) {
  // ~2 revolutions/sec → hue shifts 512 per second → period ~500ms full hue
  // Use 8-bit hue: 256 = full circle, 2 rev/s = 512 hue-units/sec
  uint8_t baseHue = (uint8_t)((elapsed * 512UL / 1000) & 0xFF);
  uint8_t hueStep = 256 / NUM_LEDS;   // 12 (~18 deg in 0-255 space)

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    uint8_t hue = baseHue + i * hueStep;
    leds[i] = hsvToRgb(hue, 240, 220);
  }
}

/* 4 ── Planning Pulse ─────────────────────────────────────────────── */
static void animPlanningPulse(uint32_t elapsed) {
  // Background: dark purple
  fill_solid(leds, NUM_LEDS, CRGB(20, 0, 30));

  // 2 pulses per second → period 500ms
  uint16_t periodMs = 500;
  uint8_t phase = (uint8_t)((elapsed % periodMs) * 256UL / periodMs);

  // Pulse radiates from center (index 9-10) outward
  uint8_t center = NUM_LEDS / 2;   // 10

  // The "pulse front" position: 0..center mapped over one period
  // phase 0-255 → position 0..center
  uint8_t frontPos = (uint16_t)phase * center / 256;

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    uint8_t dist;
    if (i >= center) {
      dist = i - center;
    } else {
      dist = center - 1 - i;
    }

    if (dist <= frontPos) {
      // Brightness falls off behind the front
      uint8_t behind = frontPos - dist;
      uint8_t bright;
      if (behind < 3) {
        bright = 255 - behind * 70;   // 255, 185, 115
      } else {
        bright = 0;
      }
      if (bright > 20) {
        leds[i] = CRGB(bright, bright, bright);
      }
    }
  }
}

/* 5 ── Approach Ramp ──────────────────────────────────────────────── */
static void animApproachRamp(uint32_t elapsed) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Fill progressively based on animSpeed and elapsed time
  // Speed 1-10: at speed 5, fill one LED every 200ms
  // At speed 10, fill one LED every 50ms; at speed 1, one every 500ms
  uint16_t msPerLed = 550 - (uint16_t)animSpeed * 50;   // 500..50
  uint8_t fillCount = (uint8_t)(elapsed / msPerLed);
  if (fillCount > NUM_LEDS) fillCount = NUM_LEDS;

  for (uint8_t i = 0; i < fillCount; i++) {
    // Brighter as we fill more (closer to object)
    uint8_t bright = 60 + (uint16_t)fillCount * 195 / NUM_LEDS;
    leds[i] = CRGB(0, bright, 0);
  }
}

/* 6 ── Grip Fill ──────────────────────────────────────────────────── */
static void animGripFill(uint32_t elapsed) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Average force across 3 FSRs → determines how many LEDs fill
  uint16_t avgForce = ((uint16_t)forceLevel[0] + forceLevel[1] + forceLevel[2]) / 3;

  // Fill from both ends toward center proportional to force
  // fillCount per side: 0..(NUM_LEDS/2)
  uint8_t halfFill = (uint8_t)((uint16_t)avgForce * (NUM_LEDS / 2) / 255);

  for (uint8_t i = 0; i < halfFill; i++) {
    uint8_t bright = 80 + (uint16_t)avgForce * 175 / 255;
    CRGB col = CRGB(bright, 0, 0);
    leds[i] = col;                         // left side
    leds[NUM_LEDS - 1 - i] = col;          // right side
  }

  // Overlay individual FSR force as brightness variation
  // FSR 0 → LEDs 0-6, FSR 1 → LEDs 7-12, FSR 2 → LEDs 13-19
  for (uint8_t fsr = 0; fsr < 3; fsr++) {
    uint8_t start = fsr * 7;
    uint8_t end   = (fsr == 2) ? NUM_LEDS : (fsr + 1) * 7;
    for (uint8_t i = start; i < end; i++) {
      if (leds[i].r > 0) {
        // Tint based on individual FSR
        leds[i] = scaleBrightness(leds[i],
                    128 + (uint16_t)forceLevel[fsr] * 127 / 255);
      }
    }
  }
}

/* 7 ── Hold Steady ────────────────────────────────────────────────── */
static void animHoldSteady(uint32_t elapsed) {
  // Gentle breathing at 0.5Hz → period 2000ms
  // +-20% brightness → base 204, amplitude 51 (in 0-255 scale: 80% of 255 = 204)
  uint8_t phase = (uint8_t)((elapsed % 2000UL) * 256UL / 2000);
  uint8_t sinVal = sin8(phase);  // 0-255
  // Map to 204 +/- 51:  center=204, swing=51
  // sinVal 0→153, 128→204, 255→255
  uint8_t bright = 153 + (uint16_t)sinVal * 102 / 255;

  CRGB col = scaleBrightness(materialCol, bright);
  fill_solid(leds, NUM_LEDS, col);
}

/* 8 ── Release Fade ───────────────────────────────────────────────── */
static void animReleaseFade(uint32_t elapsed) {
  // Exponential fade over 1500ms
  if (!releaseFadeInit) {
    memcpy(releaseFadeSnap, leds, sizeof(leds));
    releaseFadeInit = true;
    releaseFadeStart = millis();
  }

  uint32_t dt = millis() - releaseFadeStart;
  if (dt >= 1500) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    return;
  }

  // Exponential: brightness = 255 * (1 - t/1500)^3
  // Use integer: fraction = (1500-dt)/1500, cube it
  // Simplified: scale = ((1500-dt) * 255 / 1500)
  uint16_t lin = (uint16_t)((1500UL - dt) * 255UL / 1500);
  // Cube for exponential feel: scale = lin^3 / 255^2
  uint8_t scale8val = (uint8_t)((uint32_t)lin * lin / 255 * lin / 255 / 255);
  // Simpler approach: just use quadratic
  scale8val = (uint8_t)((uint32_t)lin * lin / 255);

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = scaleBrightness(releaseFadeSnap[i], (uint8_t)scale8val);
  }
}

/* 9 ── Error Strobe ───────────────────────────────────────────────── */
static void animErrorStrobe(uint32_t elapsed) {
  // 5Hz → 200ms cycle, alternating red/white each 100ms
  bool phase = ((elapsed / 100) & 1) == 0;
  if (phase) {
    fill_solid(leds, NUM_LEDS, CRGB(255, 0, 0));
  } else {
    fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
  }
}

/* 10 ── Tap Flash ─────────────────────────────────────────────────── */
static void animTapFlash(uint32_t elapsed) {
  if (elapsed < 50) {
    // Bright white flash
    fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
  } else {
    // Simplified acoustic spectrum visualization
    // First 10 LEDs = frequency bins, rest dark
    fill_solid(leds, NUM_LEDS, CRGB::Black);

    // Generate a decaying "spectrum" that settles over ~1 second
    uint32_t decayElapsed = elapsed - 50;
    uint8_t decay;
    if (decayElapsed > 2000) {
      decay = 30;   // resting level
    } else {
      decay = 255 - (uint8_t)(decayElapsed * 225UL / 2000);
    }

    for (uint8_t i = 0; i < 10; i++) {
      // Simulate spectral energy: peaks at bins 2-3 and 7 (typical tap)
      uint8_t energy;
      if (i == 2 || i == 3) {
        energy = decay;
      } else if (i == 7) {
        energy = scale8(decay, 200);
      } else if (i == 1 || i == 4) {
        energy = scale8(decay, 140);
      } else {
        energy = scale8(decay, 70);
      }

      // Color: warm (left/low freq) to cool (right/high freq)
      // Hue: 0 (red) at i=0 → 160 (blue) at i=9
      uint8_t hue = (uint16_t)i * 160 / 9;
      leds[i] = hsvToRgb(hue, 220, energy);
    }
  }
}

/* ══════════════════════════════════════════════════════════════════════
 *  ANIMATION DISPATCHER
 * ══════════════════════════════════════════════════════════════════════ */

static void runAnimation(AnimID anim, uint32_t elapsed) {
  switch (anim) {
    case ANIM_HEARTBEAT:       animHeartbeat(elapsed);      break;
    case ANIM_SEARCH_SWEEP:    animSearchSweep(elapsed);    break;
    case ANIM_DETECTED_FLASH:  animDetectedFlash(elapsed);  break;
    case ANIM_RAINBOW:         animRainbow(elapsed);        break;
    case ANIM_PLANNING_PULSE:  animPlanningPulse(elapsed);  break;
    case ANIM_APPROACH_RAMP:   animApproachRamp(elapsed);   break;
    case ANIM_GRIP_FILL:       animGripFill(elapsed);       break;
    case ANIM_HOLD_STEADY:     animHoldSteady(elapsed);     break;
    case ANIM_RELEASE_FADE:    animReleaseFade(elapsed);    break;
    case ANIM_ERROR_STROBE:    animErrorStrobe(elapsed);    break;
    case ANIM_TAP_FLASH:       animTapFlash(elapsed);       break;
    default:                   break;
  }
}

/* ══════════════════════════════════════════════════════════════════════
 *  COMMAND PROCESSING
 * ══════════════════════════════════════════════════════════════════════ */

static void startAnimation(AnimID id, uint8_t speed) {
  if (id == curAnim && !solidMode) return;   // already running

  // Snapshot current frame for crossfade
  memcpy(crossfadeBuf, leds, sizeof(leds));
  crossfading    = true;
  crossfadeAlpha = 0;
  crossfadeStart = millis();

  solidMode  = false;
  curAnim    = id;
  animSpeed  = (speed < 1) ? 1 : ((speed > 10) ? 10 : speed);
  animStartMs = millis();

  // Reset sub-state for specific animations
  detFlashDone   = false;
  tapFlashDone   = false;
  releaseFadeInit = false;
}

static void processPacket(const uint8_t *pkt) {
  uint8_t cmd = pkt[1];
  uint8_t p1  = pkt[2];
  uint8_t p2  = pkt[3];
  uint8_t p3  = pkt[4];

  switch (cmd) {
    case CMD_SOLID: {
      // Snapshot for crossfade
      memcpy(crossfadeBuf, leds, sizeof(leds));
      crossfading    = true;
      crossfadeAlpha = 0;
      crossfadeStart = millis();

      solidMode  = true;
      curAnim    = ANIM_NONE;
      solidColor = CRGB(p1, p2, p3);
      fill_solid(leds, NUM_LEDS, solidColor);
      break;
    }

    case CMD_ANIMATE: {
      if (p1 <= 10) {
        startAnimation((AnimID)p1, p2);
      }
      break;
    }

    case CMD_PIXEL: {
      // Set single LED: P1=index, P2=R, P3=G, blue=0
      if (p1 < NUM_LEDS) {
        leds[p1] = CRGB(p2, p3, 0);
      }
      break;
    }

    case CMD_MATERIAL: {
      if (p1 < NUM_MATERIALS) {
        materialId   = p1;
        materialConf = p2;
        materialCol  = materialColors[p1];
        // Modulate by confidence
        materialCol  = scaleBrightness(materialCol,
                         128 + (uint16_t)p2 * 127 / 255);
      }
      break;
    }

    case CMD_HEARTBEAT: {
      uint16_t bpm = (uint16_t)p1 * 256 + p2;
      if (bpm >= 100 && bpm <= 3000) {   // 10.0 - 300.0 BPM sane range
        heartbeatBpmX10 = bpm;
      }
      break;
    }

    case CMD_FORCE_MAP: {
      if (p1 < 3) {
        forceLevel[p1] = p2;
      }
      break;
    }

    default:
      break;   // unknown command, ignore
  }
}

/* ══════════════════════════════════════════════════════════════════════
 *  UART RECEIVE (non-blocking, byte-by-byte)
 * ══════════════════════════════════════════════════════════════════════ */

static void uartPoll(void) {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    uint32_t now = millis();

    // If too much time elapsed since last byte, reset
    if (rxIdx > 0 && (now - rxLastByte) > UART_TIMEOUT_MS) {
      rxIdx = 0;
    }
    rxLastByte = now;

    if (rxIdx == 0) {
      // Waiting for sync byte
      if (b == SYNC_BYTE) {
        rxBuf[0] = b;
        rxIdx = 1;
      }
      // else discard
    } else {
      rxBuf[rxIdx++] = b;
      if (rxIdx >= PACKET_LEN) {
        processPacket(rxBuf);
        rxIdx = 0;
      }
    }

    // Safety: prevent buffer overrun (should not happen, but defensive)
    if (rxIdx >= PACKET_LEN) {
      rxIdx = 0;
    }
  }
}

/* ══════════════════════════════════════════════════════════════════════
 *  CROSSFADE LOGIC
 * ══════════════════════════════════════════════════════════════════════ */

static void applyCrossfade(void) {
  if (!crossfading) return;

  uint32_t dt = millis() - crossfadeStart;
  if (dt >= CROSSFADE_MS) {
    crossfading = false;
    crossfadeAlpha = 255;
    return;
  }

  crossfadeAlpha = (uint8_t)(dt * 255UL / CROSSFADE_MS);

  // Blend: leds = crossfadeBuf * (255 - alpha) + leds * alpha
  // We need a temp because leds already has the new frame
  CRGB temp[NUM_LEDS];
  memcpy(temp, leds, sizeof(leds));
  blendArrays(leds, crossfadeBuf, temp, crossfadeAlpha, NUM_LEDS);
}

/* ══════════════════════════════════════════════════════════════════════
 *  SETUP & LOOP
 * ══════════════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MASTER_BRIGHT);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 800);   // safety limit

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // Boot sequence: quick blue sweep to indicate ready
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 120);
    FastLED.show();
    delay(25);   // delay OK here, it's one-time boot
  }
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();

  // Start with idle heartbeat
  curAnim    = ANIM_HEARTBEAT;
  animStartMs = millis();
  lastFrameMs = millis();
}

void loop() {
  // ── UART receive ──
  uartPoll();

  // ── Frame rate limiter ──
  uint32_t now = millis();
  if ((now - lastFrameMs) < FRAME_INTERVAL_MS) return;
  lastFrameMs = now;

  // ── Run animation or hold solid ──
  if (!solidMode && curAnim != ANIM_NONE) {
    uint32_t elapsed = now - animStartMs;
    runAnimation(curAnim, elapsed);
  }

  // ── Crossfade blending ──
  applyCrossfade();

  // ── Push to strip ──
  FastLED.show();
}
