// =====================================================================
//  MODAL FEEDBACK – pure acoustic feedback with the logic of Modal States
//  Feedback workshop board · Leon Eckard 2026
//
//  Microphone -> lowpass -> gain -> speaker -> air -> microphone.
//  The gain grows until the loop starts to sing, and slows down the closer
//  the level gets to its target. While singing, the loop gets tired.
//  Once fully tired, the gain falls, the loop goes quiet and rests until
//  its fatigue is gone. Then it grows again.
//  The same rule as the strings of Moon, Eye and Wood.
//
//  Pot (fully down = dark): lowpass 60..500 Hz
//  Optional pots on the Joker header: A1 = target level, A2 = tempo
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
#define PIN_A1    36   // Joker A1
#define PIN_A2    39   // Joker A2
#define PIN_LED   2    // blue LED on the DevKit

// ---------- Behaviour, as in logic.scd ----------
float GROWTH_RATE       = 0.05f; // gain per second at the start (base time = 1 / GROWTH_RATE)
const float SENSE_RATIO = 0.8f;   // endurance: how long it can sing
const float FALL_RATIO  = 0.03f;   // how fast the gain falls when tired
const float REST_RATIO  = 0.1f;   // how long it rests
float AMP_MAX           = 0.7f;   // target output level, 0..1
const float VOICE_FLOOR = 0.1f;   // below this it does not count as singing
const float MAX_GAIN    = 400.0f; // amplification at gain = 1 (the mic is quiet)

// After resting, restart close to the gain where the loop began to sing,
// instead of growing all the way up from zero. Avoids long silences.
const bool  REMEMBER_ONSET = false;
const float REGROW_FROM    = 0.7f; // restart at 70 % of the remembered onset gain

// ---------- Sound ----------
const float LPF_MAX   = 2000.0f;   // pot fully up
const float LPF_MIN   = 60.0f;    // pot fully down
const float LPF_OPEN  = 0.3f;     // with no gain the filter sits at 30 % of the pot value
const float FILTER_Q  = 1.25f;    // like RLPF with rq 0.8
const float HPF_HZ    = 40.0f;    // removes rumble and DC
const float VOLUME    = 0.6f;
const bool  POT_INVERT = true;    // on this board, fully down reads the highest value
const bool  JOKER_POTS = false;   // set to true only if pots are connected to A1 and A2

// ---------- Audio ----------
const int SR = 44100;
const int N  = 128;               // frames per block, about 2.9 ms
const float DT = (float)N / SR;
int32_t rxBuf[N * 2], txBuf[N * 2];
i2s_chan_handle_t txChan, rxChan;

// Two-pole lowpass (TPT state variable filter)
struct SVF { float ic1 = 0, ic2 = 0, a1 = 0, a2 = 0, a3 = 0; };
SVF lp1, lp2;
void setLP(SVF &f, float fc) {
  float g = tanf(PI * fc / SR), k = 1.0f / FILTER_Q;
  f.a1 = 1.0f / (1.0f + g * (g + k)); f.a2 = g * f.a1; f.a3 = g * f.a2;
}
static inline float runLP(SVF &f, float x) {
  float v3 = x - f.ic2;
  float v1 = f.a1 * f.ic1 + f.a2 * v3;
  float v2 = f.ic2 + f.a2 * f.ic1 + f.a3 * v3;
  f.ic1 = 2.0f * v1 - f.ic1; f.ic2 = 2.0f * v2 - f.ic2;
  return v2;
}

// ---------- State ----------
float gain = 0, fatigue = 0, amp = 0, potS = 0.5f;
float onsetGain = 0;              // gain at which the loop last started to sing
bool  tired = false, wasTired = false, wasVoiced = false;
float hpX = 0, hpY = 0;
const float hpCoef = expf(-2.0f * PI * HPF_HZ / SR);
const float ampAtt = 1.0f - expf(-1.0f / (0.05f * SR));   // like Amplitude.kr(..., 0.05, 0.3)
const float ampRel = 1.0f - expf(-1.0f / (0.3f * SR));
uint32_t blockCount = 0;

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
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);
  setupI2S();
  Serial.println("Modal Feedback ready.");
}

void loop() {
  // reading one block of microphone data also sets the timing
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);

  // ---- Controls ----
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.05f;
  float lpfHigh = LPF_MIN * powf(LPF_MAX / LPF_MIN, potS);
  if (JOKER_POTS) {
    AMP_MAX     = 0.1f + 0.8f * analogRead(PIN_A1) / 4095.0f;
    GROWTH_RATE = 0.05f + 1.0f * analogRead(PIN_A2) / 4095.0f;
  }

  // ---- All times follow from the growth rate, as in logic.scd ----
  float base      = 1.0f / GROWTH_RATE;
  float senseTime = base * SENSE_RATIO;
  float fallTime  = base * FALL_RATIO;
  float restTime  = base * REST_RATIO;

  // ---- The string: grow, tire, fall, rest ----
  float ratio = fminf(amp / AMP_MAX, 1.0f);
  bool  voiced = ratio > VOICE_FLOOR;
  float voice = voiced ? 1.0f : 0.0f;
  float t     = tired ? 1.0f : 0.0f;

  // remember the gain at which the loop started to sing
  if (voiced && !wasVoiced && !tired) onsetGain = gain;
  wasVoiced = voiced;

  float growth = GROWTH_RATE * (1.0f - amp / AMP_MAX);          // slower the closer it gets to the target
  gain    += growth * DT * (1.0f - t) - gain * t * DT / fallTime;
  gain     = fminf(fmaxf(gain, 0.0f), 1.0f);
  fatigue += ratio * voice * DT / senseTime * (1.0f - t) - t * DT / restTime;
  fatigue  = fminf(fmaxf(fatigue, 0.0f), 1.0f);
  tired    = (fatigue >= 1.0f) || (tired && fatigue > 0.0f);

  // rest is over: restart just below the remembered onset
  if (REMEMBER_ONSET && wasTired && !tired) gain = fmaxf(gain, onsetGain * REGROW_FROM);
  wasTired = tired;

  // ---- The lowpass opens with the gain, up to the pot value ----
  float cutoff = lpfHigh * LPF_OPEN * powf(1.0f / LPF_OPEN, gain);   // exponential, 30 % to 100 %
  setLP(lp1, cutoff);
  setLP(lp2, lpfHigh);
  float g = gain * MAX_GAIN;

  for (int i = 0; i < N; i++) {
    float x = (rxBuf[2 * i] >> 8) / 8388608.0f;              // 24 bit in the top of a 32 bit word
    float y = hpCoef * (hpY + x - hpX);  hpX = x; hpY = y;   // highpass against rumble
    float s = runLP(lp1, y) * g;
    s = tanhf(s);                                            // soft limiter
    s = runLP(lp2, s);                                       // everything stays below the lowpass
    float r = fabsf(s);
    amp += (r - amp) * (r > amp ? ampAtt : ampRel);
    int32_t o = (int32_t)(s * VOLUME * 2147483000.0f);
    txBuf[2 * i] = o;                                        // left
    txBuf[2 * i + 1] = o;                                    // right, same signal
  }

  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);
  digitalWrite(PIN_LED, ratio > 0.3f ? HIGH : LOW);          // on while it sings

  // ---- Monitor, four times per second (115200 baud) ----
  if (++blockCount % 86 == 0) {
    Serial.printf("%s | gain %.3f (onset %.3f) | level %.2f / %.2f | fatigue %.2f | filter %3.0f Hz (max %3.0f)\n",
                  tired ? "RESTING" : "growing", gain, onsetGain, amp, AMP_MAX, fatigue, cutoff, lpfHigh);
  }
}
