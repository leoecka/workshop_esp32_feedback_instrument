// =====================================================================
//  SELF-FM – a sawtooth that modulates itself through the air
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  The microphone hears its own speaker, and exactly this signal bends
//  the pitch of the sawtooth: frequency modulation with itself, through
//  room, distance and housing. Every movement in the room changes the sound.
//
//  Pot (fully down = 0):
//    0          silent
//    0 .. 1/2   100 Hz, slowly gets louder, lowpass opens 120 -> 2000 Hz
//    1/2 .. 1   pitch rises from 100 Hz up to PITCH_MAX, filter follows
//
//  Arduino-ESP32 3.x (ESP-IDF 5), board "ESP32 Dev Module".
// =====================================================================

#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h>

// ---------- Pins of board v2 ----------
#define PIN_BCLK  26
#define PIN_WS    25
#define PIN_DOUT  22   // to AMP and LINE OUT
#define PIN_MIC   35   // data from the INMP441
#define PIN_POT   34

// ---------- Things to play with ----------
const float FM_DEPTH  = 0.5f;    // 0.10 = the mic bends the pitch by up to ±10 %
const float RING      = 0.2f;    // 0..1: also multiply by the mic signal (ring modulation)
const float MIC_GAIN  = 50.0f;   // how much the mic is amplified before modulating
const float BASE_HZ   = 100.0f;
const float PITCH_MAX = 800.0f;  // highest pitch with the pot fully up
const float FILTER_Q  = 1.2f;
const float VOLUME    = 0.4f;
const bool  POT_INVERT = true;   // on this board, fully down reads the highest value

// ---------- Audio ----------
const int SR = 44100;
const int N  = 128;              // frames per block, about 2.9 ms
int32_t rxBuf[N * 2], txBuf[N * 2];
i2s_chan_handle_t txChan, rxChan;

float sawPhase = 0;
float potS = 0;                  // smoothed pot
float ampNow = 0;                // volume, smoothed per sample
float ic1 = 0, ic2 = 0;          // filter state
float hpX = 0, hpY = 0;          // DC blocker state
float micLvl = 0;
uint32_t blockCount = 0;

// sawtooth correction (PolyBLEP) so it aliases less
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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  setupI2S();
  Serial.println("Self-FM ready.");
}

void loop() {
  // reading one block of microphone data also sets the timing
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);

  // ---- Pot: 0 = fully down ----
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.08f;
  float q = potS < 0.01f ? 0.0f : potS;          // really silent at the bottom

  // ---- The pot journey ----
  float ampTarget, freq, fc;
  if (q < 0.5f) {
    float h = q / 0.5f;                          // 0..1 in the lower half
    ampTarget = h * h;                           // slowly louder
    freq = BASE_HZ;
    fc = 120.0f * powf(2000.0f / 120.0f, h);     // filter opens exponentially
  } else {
    float h = (q - 0.5f) / 0.5f;                 // 0..1 in the upper half
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
    float dt = freq * (1.0f + FM_DEPTH * m) / SR;
    float saw = 2.0f * sawPhase - 1.0f - polyblep(sawPhase, dt);
    sawPhase += dt;
    if (sawPhase >= 1.0f) sawPhase -= 1.0f;

    // ---- optionally also multiply (ring modulation) ----
    saw *= (1.0f - RING) + RING * m;

    // ---- Lowpass ----
    float v3 = saw - ic2;
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
    Serial.printf("pot %.2f | %5.1f Hz | filter %5.0f Hz | volume %.2f | mic %.2f\n",
                  q, freq, fc, ampNow, micLvl);
  }
}
