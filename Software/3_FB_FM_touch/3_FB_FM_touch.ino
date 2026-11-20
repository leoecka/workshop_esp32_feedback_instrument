// =====================================================================
//  SELF-FM TOUCH – a sawtooth that modulates itself through the air,
//  with one pot and two touch points
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  The microphone hears its own speaker, and exactly this signal bends
//  the pitch of the oscillator: frequency modulation with itself, through
//  room, distance and housing. Every movement in the room changes the sound.
//
//  TOUCH: tap the solder point of D32 on top of the ESP board -> next pot function
//         tap the solder point of D4 (other side of the board) -> previous pot function
//  The blue LED lights while you touch, then blinks the number of the new function.
//  At start-up it blinks three times as a test. With DEBUG_TOUCH the serial monitor
//  shows the touch reading, so TOUCH_RATIO can be adjusted.
//
//  POT FUNCTIONS (fully down = 0):
//    1 JOURNEY  0: silent, up to 1/2: louder and brighter at 100 Hz,
//               above 1/2: pitch rises up to PITCH_MAX
//    2 SHAPE    sawtooth -> square, fluid
//    3 FM       how strongly the mic bends the pitch, 0..1
//    4 RING     how strongly the mic multiplies the sound, 0..1
//  After switching, a function keeps its value until you turn the pot
//  across it (pickup), so nothing jumps.
//
//  Arduino-ESP32 3.x (ESP-IDF 5), board "ESP32 Dev Module".
// =====================================================================

#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// ---------- Pins of board v2 ----------
#define PIN_BCLK   26
#define PIN_WS     25
#define PIN_DOUT   22   // to AMP and LINE OUT
#define PIN_MIC    35   // data from the INMP441
#define PIN_POT    34
#define PIN_LED    2    // blue LED on the DevKit
#define PIN_NEXT   4   // touch T9: next function
#define PIN_PREV   32    // touch T0: previous function (other side of the board)

// ---------- Things to play with ----------
const float FM_START   = 0.5f;    // FM depth at start-up (0..1)
const float RING_START = 0.2f;    // ring modulation at start-up (0..1)
const float MIC_GAIN   = 50.0f;   // how much the mic is amplified before modulating
const float BASE_HZ    = 100.0f;
const float PITCH_MAX  = 800.0f;  // highest pitch with JOURNEY fully up
const float FILTER_Q   = 1.2f;
const float VOLUME     = 0.4f;
const bool  POT_INVERT = true;    // on this board, fully down reads the highest value
const float TOUCH_RATIO = 0.85f;  // touched when the reading drops below 85 % of its resting value (a light touch is enough)
const bool  DEBUG_TOUCH = true;   // show touch readings in the serial monitor
const uint32_t TAP_GAP_MS = 300;  // ignore new taps for this long (against double switching)
const uint32_t BLINK_MS   = 250;  // LED on and off time when showing the function number
const float PICKUP     = 0.03f;   // how close the pot must come to take over a value

// ---------- Audio ----------
const int SR = 44100;
const int N  = 128;               // frames per block, about 2.9 ms
int32_t rxBuf[N * 2], txBuf[N * 2];
i2s_chan_handle_t txChan, rxChan;

// ---------- Pot functions ----------
enum Mode { JOURNEY = 0, SHAPE = 1, FM = 2, RING = 3, MODES = 4 };
const char *MODE_NAMES[] = { "JOURNEY", "SHAPE", "FM", "RING" };
int   mode = JOURNEY;
float value[MODES];               // current value of every function, 0..1
bool  armed = true;               // has the pot taken over the current function yet?

// ---------- State ----------
float phase = 0;
float potS = 0;                   // smoothed pot
float ampNow = 0;
float ic1 = 0, ic2 = 0;           // filter state
float hpX = 0, hpY = 0;           // DC blocker state
float micLvl = 0;
uint32_t blockCount = 0;

// touch: index 0 = next (D32), index 1 = previous (D4)
const int TOUCH_PINS[2] = { PIN_NEXT, PIN_PREV };
float touchBase[2] = { 0, 0 }, touchLast[2] = { 0, 0 };
bool  touchDown[2] = { false, false };
uint8_t touchCount[2] = { 0, 0 };
uint32_t lastTapAt = 0;

// LED blink pattern
int ledBlinksLeft = 0;
uint32_t ledNextAt = 0;
bool ledOn = false;

// band-limited corrections for saw and square (PolyBLEP)
static inline float polyblep(float t, float dt) {
  if (t < dt)        { t /= dt;             return t + t - t * t - 1.0f; }
  if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
  return 0.0f;
}

void setupI2S() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = N;
  chanCfg.auto_clear    = true;
  // one port, two channels: send and receive share the same clocks
  ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, &txChan, &rxChan));
  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SR),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)PIN_BCLK,
      .ws   = (gpio_num_t)PIN_WS,
      .dout = (gpio_num_t)PIN_DOUT,
      .din  = (gpio_num_t)PIN_MIC,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txChan, &stdCfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rxChan, &stdCfg));
  ESP_ERROR_CHECK(i2s_channel_enable(txChan));
  ESP_ERROR_CHECK(i2s_channel_enable(rxChan));
}

// switch the pot to another function
void setMode(int m) {
  mode = (m + MODES) % MODES;
  armed = false;                                  // wait until the pot reaches the stored value
  ledBlinksLeft = (mode + 1) * 2;                 // on and off for each blink
  ledNextAt = millis() + 400;                     // short pause after the touch light
  Serial.printf("-> %d %s (turn the pot across %.2f to take over)\n", mode + 1, MODE_NAMES[mode], value[mode]);
}

// read one touch point; returns true once per tap
bool tapped(int i) {
  float v = touchRead(TOUCH_PINS[i]);
  touchLast[i] = v;
  bool below = v < touchBase[i] * TOUCH_RATIO;
  if (below) { if (touchCount[i] < 255) touchCount[i]++; }
  else       { touchCount[i] = 0; touchBase[i] += (v - touchBase[i]) * 0.05f; }   // slowly follow the resting value
  bool tap = false;
  if (!touchDown[i] && touchCount[i] >= 2) {                                    // stable for two readings
    touchDown[i] = true;
    if (millis() - lastTapAt > TAP_GAP_MS) { tap = true; lastTapAt = millis(); }
  }
  if (touchDown[i] && !below) touchDown[i] = false;                             // released
  return tap;
}

void updateLed() {
  if (touchDown[0] || touchDown[1]) { digitalWrite(PIN_LED, HIGH); ledOn = false; return; }   // lights while touched
  if (ledBlinksLeft > 0) {
    if (millis() >= ledNextAt) {
      ledOn = !ledOn;
      ledBlinksLeft--;
      ledNextAt = millis() + BLINK_MS;
      if (ledBlinksLeft == 0) ledOn = false;
    }
    digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);

  // LED test: three blinks
  for (int k = 0; k < 3; k++) { digitalWrite(PIN_LED, HIGH); delay(150); digitalWrite(PIN_LED, LOW); delay(150); }

  // resting values of the touch points (do not touch them while starting)
  for (int i = 0; i < 2; i++) {
    float s = 0;
    for (int k = 0; k < 16; k++) { s += touchRead(TOUCH_PINS[i]); delay(5); }
    touchBase[i] = s / 16.0f;
  }

  // start values
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS = p;
  value[JOURNEY] = p;
  value[SHAPE]   = 0.0f;
  value[FM]      = FM_START;
  value[RING]    = RING_START;

  setupI2S();
  Serial.printf("Self-FM touch ready. Resting values: D32 %.0f (next), D4 %.0f (previous)\n", touchBase[0], touchBase[1]);
  setMode(JOURNEY);
  armed = true;                                   // at start the pot already sits on JOURNEY
}

void loop() {
  // reading one block of microphone data also sets the timing
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);

  // ---- Touch, about every 15 ms ----
  if (blockCount % 5 == 0) {
    if (tapped(0)) setMode(mode + 1);
    if (tapped(1)) setMode(mode - 1);
  }
  updateLed();
  if (DEBUG_TOUCH && blockCount % 86 == 0) {
    Serial.printf("touch D32 next: %4.0f / %4.0f -> %4.0f %-7s | D4 back: %4.0f / %4.0f -> %4.0f %s\n",
                  touchLast[0], touchBase[0], touchBase[0] * TOUCH_RATIO, touchDown[0] ? "TOUCHED" : "-",
                  touchLast[1], touchBase[1], touchBase[1] * TOUCH_RATIO, touchDown[1] ? "TOUCHED" : "-");
  }

  // ---- Pot, with pickup ----
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.08f;
  if (!armed && fabsf(potS - value[mode]) < PICKUP) armed = true;
  if (armed) value[mode] = potS;

  float shape = value[SHAPE];
  float fmDepth = value[FM];
  float ring = value[RING];

  // ---- JOURNEY: loudness, filter and pitch ----
  float q = value[JOURNEY] < 0.01f ? 0.0f : value[JOURNEY];   // really silent at the bottom
  float ampTarget, freq, fc;
  if (q < 0.5f) {
    float h = q / 0.5f;
    ampTarget = h * h;                           // slowly louder
    freq = BASE_HZ;
    fc = 120.0f * powf(2000.0f / 120.0f, h);     // filter opens exponentially
  } else {
    float h = (q - 0.5f) / 0.5f;
    ampTarget = 1.0f;
    freq = BASE_HZ * powf(PITCH_MAX / BASE_HZ, h);
    fc = 2000.0f * (freq / BASE_HZ);             // filter follows the pitch
  }
  if (fc > 9000.0f) fc = 9000.0f;

  // lowpass coefficients, once per block (TPT state variable filter)
  float g  = tanf(PI * fc / SR);
  float k  = 1.0f / FILTER_Q;
  float a1 = 1.0f / (1.0f + g * (g + k));
  float a2 = g * a1, a3 = g * a2;
  float ampStep = (ampTarget - ampNow) / N;      // glide across the block, no clicks

  for (int i = 0; i < N; i++) {
    // ---- Mic: remove DC, soft-limit to ±1 ----
    float x = (rxBuf[2 * i] >> 8) / 8388608.0f; // 24 bit in the top of a 32 bit word
    float y = x - hpX + 0.995f * hpY;  hpX = x; hpY = y;
    float m = tanhf(y * MIC_GAIN);
    float r = fabsf(m);
    micLvl += (r - micLvl) * 0.001f;

    // ---- FM: the mic bends the pitch ----
    float dt = freq * (1.0f + fmDepth * m) / SR;
    if (dt < 0.0f) dt = 0.0f;

    // ---- SHAPE: sawtooth and square, mixed fluidly ----
    float saw = 2.0f * phase - 1.0f - polyblep(phase, dt);
    float p2  = phase + 0.5f; if (p2 >= 1.0f) p2 -= 1.0f;
    float sq  = (phase < 0.5f ? 1.0f : -1.0f) + polyblep(phase, dt) - polyblep(p2, dt);
    float osc = saw + (sq - saw) * shape;
    phase += dt;
    if (phase >= 1.0f) phase -= 1.0f;

    // ---- RING: also multiply by the mic ----
    osc *= (1.0f - ring) + ring * m;

    // ---- Lowpass ----
    float v3 = osc - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2.0f * v1 - ic1;
    ic2 = 2.0f * v2 - ic2;

    ampNow += ampStep;
    float out = v2 * ampNow * VOLUME;
    if (out > 1.0f) out = 1.0f; else if (out < -1.0f) out = -1.0f;
    int32_t s = (int32_t)(out * 2147483000.0f);
    txBuf[2 * i] = s;                            // left
    txBuf[2 * i + 1] = s;                        // right, same signal
  }

  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);

  // ---- Monitor, twice per second (115200 baud) ----
  if (++blockCount % 172 == 0) {
    Serial.printf("pot: %d %-7s%s | journey %.2f (%5.1f Hz) shape %.2f fm %.2f ring %.2f | mic %.2f\n",
                  mode + 1, MODE_NAMES[mode], armed ? "" : " (turn to pick up)",
                  value[JOURNEY], freq, shape, fmDepth, ring, micLvl);
  }
}