
// =====================================================================
//  MODAL FEEDBACK SHIFT DELAY – acoustic feedback with the logic of Modal States,
//  plus a pitch shifter and a delay inside the loop
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  Microphone -> lowpass -> gain -> pitch shift -> delay -> speaker -> air -> microphone.
//  The gain grows until the loop starts to sing, and slows down the closer
//  the level gets to its target. While singing, the loop gets tired.
//  Once fully tired, the gain falls, the loop goes quiet and rests until
//  its fatigue is gone. Then it grows again.
//  The pitch shifter and the delay sit inside the loop: with a pitch other
//  than 1 the feedback climbs or sinks with every round, with a delay it
//  turns into echoes that grow, tire and rest.
//
//  POT          lowpass 60..LPF_MAX Hz (fully down = dark)
//  hold D4      the pot sets the PITCH 0.9..1.1 (middle = 1, no shift)
//  hold D32     the pot sets the DELAY 0..1000 ms
//  hold both    resets the PITCH to exactly 1 (the LED flashes)
//  After changing what the pot does, a value only follows the pot once you
//  turn across it (pickup), so nothing jumps.
//  LED: on while the loop sings, blinks slowly while D4 is held, fast while D32 is held.
//  The pitch shifter aligns each new grain with the one that is playing (WSOLA),
//  so held tones do not pulse or click rhythmically.
//
//  Arduino-ESP32 3.x (ESP-IDF 5), board "ESP32 Dev Module".
// =====================================================================
 
#include <Arduino.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <math.h>
 
// ---------- Pins of board v2 ----------
#define PIN_BCLK  26
#define PIN_WS    25
#define PIN_DOUT  22   // to AMP and LINE OUT
#define PIN_MIC   35   // data from the INMP441
#define PIN_POT   34
#define PIN_LED   2    // blue LED on the DevKit
#define PIN_DELAY 32   // touch T9: hold to set the delay
#define PIN_PITCH 4    // touch T0: hold to set the pitch
 
// ---------- Behaviour, as in logic.scd ----------
float GROWTH_RATE       = 0.05f;  // gain per second at the start (base time = 1 / GROWTH_RATE)
const float SENSE_RATIO = 0.8f;   // endurance: how long it can sing
const float FALL_RATIO  = 0.03f;  // how fast the gain falls when tired
const float REST_RATIO  = 0.1f;   // how long it rests
float AMP_MAX           = 0.7f;   // target output level, 0..1
const float VOICE_FLOOR = 0.1f;   // below this it does not count as singing
const float MAX_GAIN    = 400.0f; // amplification at gain = 1 (the mic is quiet)
 
// After resting, restart close to the gain where the loop began to sing,
// instead of growing all the way up from zero. Avoids long silences.
const bool  REMEMBER_ONSET = false;
const float REGROW_FROM    = 0.7f; // restart at 70 % of the remembered onset gain
 
// ---------- Sound ----------
const float LPF_MAX   = 2000.0f;  // pot fully up
const float LPF_MIN   = 60.0f;    // pot fully down
const float LPF_OPEN  = 0.3f;     // with no gain the filter sits at 30 % of the pot value
const float FILTER_Q  = 1.25f;    // like RLPF with rq 0.8
const float HPF_HZ    = 40.0f;    // removes rumble and DC
const float VOLUME    = 0.6f;
const bool  POT_INVERT = true;    // on this board, fully down reads the highest value
 
// ---------- Pitch shift and delay ----------
const float PITCH_MIN    = 0.9f;  // pitch with the pot fully down (while D4 is held)
const float PITCH_MAX    = 1.1f;  // pitch with the pot fully up
const float DELAY_MAX_MS = 1000.0f;
const bool  DELAY_SQUARE = true;  // true: finer control of short delays (middle = 250 ms), false: linear
const int   SHIFT_WINDOW = 2048;  // pitch shifter window in samples (about 46 ms)
const int   SHIFT_SEARCH = 192;   // how far a new grain may be moved to match the playing one (samples)
const int   SHIFT_CORR   = 192;   // how many samples are compared for the match
 
// ---------- Touch ----------
const float TOUCH_RATIO = 0.85f;  // touched below 85 % of the resting value
const bool  DEBUG_TOUCH = false;  // show touch readings in the serial monitor
const float PICKUP      = 0.03f;
 
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
 
// ---------- Pitch shifter: two read heads sliding through a short buffer ----------
const int PS_LEN = 4096;          // must be a power of two and larger than SHIFT_WINDOW
float psBuf[PS_LEN];
int   psWrite = 0;
float psPhase = 0;                // 0..1, position of the heads inside the window
float psMix = 0;                  // 0 = bypass (pitch 1), 1 = shifted
float psOff[2] = { 0, 0 };        // extra delay of each head, chosen when it starts a new grain
const float PS_BASE = SHIFT_CORR + SHIFT_SEARCH + 8;   // shortest delay, leaves room for matching
float psRef[SHIFT_CORR];
 
static inline float frac1(float x) { return x - floorf(x); }
 
// a head starts a new grain: find the offset that is in phase with the other head
void psAlign(int h, float phase) {
  int o = 1 - h;
  int dOther = (int)roundf(PS_BASE + frac1(phase + 0.5f * o) * SHIFT_WINDOW + psOff[o]);
  for (int n = 0; n < SHIFT_CORR; n++) psRef[n] = psBuf[(psWrite - dOther + n) & (PS_LEN - 1)];
  int dNew = (int)roundf(PS_BASE + frac1(phase + 0.5f * h) * SHIFT_WINDOW);
  float best = -1e30f; int bestK = 0;
  for (int k = -SHIFT_SEARCH; k <= SHIFT_SEARCH; k += 2) {
    int start = psWrite - (dNew + k);
    float c = 0;
    for (int n = 0; n < SHIFT_CORR; n++) c += psBuf[(start + n) & (PS_LEN - 1)] * psRef[n];
    if (c > best) { best = c; bestK = k; }
  }
  psOff[h] = (float)bestK;
}
 
static inline float psRead(float delaySamples) {
  float rp = psWrite - delaySamples;
  while (rp < 0) rp += PS_LEN;
  int i0 = (int)rp; float fr = rp - i0;
  float a = psBuf[i0 & (PS_LEN - 1)], b = psBuf[(i0 + 1) & (PS_LEN - 1)];
  return a + (b - a) * fr;
}
 
// ---------- Delay line ----------
int16_t *dlyBuf = nullptr;
uint32_t dlyLen = 0, dlyWrite = 0;
float dlyMaxMs = 0;
float dlySamplesS = 0;            // smoothed delay time in samples
 
// ---------- Pot with three jobs ----------
enum Job { JOB_LPF = 0, JOB_PITCH = 1, JOB_DELAY = 2, JOB_RESET = 3 };
const char *JOB_NAMES[] = { "LOWPASS", "PITCH", "DELAY", "RESET" };
float jobValue[4] = { 0.5f, 0.5f, 0.0f, 0.0f };   // pot positions 0..1 of each job (0.5 = pitch 1)
uint32_t resetFlashUntil = 0;
int   job = JOB_LPF;
bool  armed = true;
float potS = 0.5f;
 
// ---------- Touch state ----------
const int TOUCH_PINS[2] = { PIN_PITCH, PIN_DELAY };
float touchBase[2] = { 0, 0 }, touchLast[2] = { 0, 0 };
bool  touchDown[2] = { false, false };
uint8_t touchCount[2] = { 0, 0 };
 
// ---------- State ----------
float gain = 0, fatigue = 0, amp = 0;
float onsetGain = 0;              // gain at which the loop last started to sing
bool  tired = false, wasTired = false, wasVoiced = false;
float hpX = 0, hpY = 0;
const float hpCoef = expf(-2.0f * PI * HPF_HZ / SR);
const float ampAtt = 1.0f - expf(-1.0f / (0.05f * SR));   // like Amplitude.kr(..., 0.05, 0.3)
const float ampRel = 1.0f - expf(-1.0f / (0.3f * SR));
uint32_t blockCount = 0;
 
void readTouch(int i) {
  float v = touchRead(TOUCH_PINS[i]);
  touchLast[i] = v;
  bool below = v < touchBase[i] * TOUCH_RATIO;
  if (below) { if (touchCount[i] < 255) touchCount[i]++; }
  else       { touchCount[i] = 0; touchBase[i] += (v - touchBase[i]) * 0.05f; }
  if (!touchDown[i] && touchCount[i] >= 2) touchDown[i] = true;   // stable for two readings
  if (touchDown[i] && !below) touchDown[i] = false;
}
 
static inline float pitchFromPot(float p) { return PITCH_MIN + (PITCH_MAX - PITCH_MIN) * p; }   // middle = 1
static inline float delayMsFromPot(float p) { return dlyMaxMs * (DELAY_SQUARE ? p * p : p); }
 
void allocateDelay() {
  uint32_t want = (uint32_t)(DELAY_MAX_MS * 0.001f * SR) + 4;
  while (want > 1000 && !dlyBuf) {
    dlyBuf = (int16_t *)heap_caps_malloc(want * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!dlyBuf) want = want * 3 / 4;                // not enough in one piece: try shorter
  }
  if (dlyBuf) { memset(dlyBuf, 0, want * sizeof(int16_t)); dlyLen = want; }
  dlyMaxMs = dlyLen > 4 ? (dlyLen - 4) * 1000.0f / SR : 0;
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
 
  allocateDelay();
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS = p; jobValue[JOB_LPF] = p;
 
  setupI2S();
  Serial.printf("Modal Feedback shift delay ready. Longest delay %.0f ms.\n", dlyMaxMs);
  Serial.println("Pot = lowpass. Hold D4 for pitch (0.9..1.1), hold D32 for delay, hold both to reset the pitch.");
}
 
void loop() {
  // reading one block of microphone data also sets the timing
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);
 
  // ---- Touch, about every 15 ms ----
  if (blockCount % 5 == 0) { readTouch(0); readTouch(1); }
  int wantJob = (touchDown[0] && touchDown[1]) ? JOB_RESET
              : touchDown[0] ? JOB_PITCH : (touchDown[1] ? JOB_DELAY : JOB_LPF);
  if (wantJob != job) {
    if (wantJob == JOB_RESET) {                            // both held: pitch back to exactly 1
      jobValue[JOB_PITCH] = 0.5f;
      resetFlashUntil = millis() + 400;
      Serial.println("Pitch reset to 1.");
    }
    job = wantJob; armed = false;                          // the pot changes its job: pick up first
  }
 
  // ---- Pot, with pickup ----
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.05f;
  if (job != JOB_RESET) {                                  // while both are held, the pot does nothing
    if (!armed && fabsf(potS - jobValue[job]) < PICKUP) armed = true;
    if (armed) jobValue[job] = potS;
  }
 
  float lpfHigh = LPF_MIN * powf(LPF_MAX / LPF_MIN, jobValue[JOB_LPF]);
  float pitch   = pitchFromPot(jobValue[JOB_PITCH]);
  float dlyMs   = delayMsFromPot(jobValue[JOB_DELAY]);
 
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
 
  // pitch shifter: how fast the heads slide, and whether it is used at all
  const float psStep = (1.0f - pitch) / SHIFT_WINDOW;
  const float mixTarget = fabsf(pitch - 1.0f) < 0.003f ? 0.0f : 1.0f;   // exactly 1: bypass, no comb colouring
  const float dlyTarget = dlyMs * 0.001f * SR;
 
  for (int i = 0; i < N; i++) {
    float x = (rxBuf[2 * i] >> 8) / 8388608.0f;              // 24 bit in the top of a 32 bit word
    float y = hpCoef * (hpY + x - hpX);  hpX = x; hpY = y;   // highpass against rumble
    float s = runLP(lp1, y) * g;
    s = tanhf(s);                                            // soft limiter
    s = runLP(lp2, s);                                       // everything stays below the lowpass
    float r = fabsf(s);
    amp += (r - amp) * (r > amp ? ampAtt : ampRel);
 
    // ---- Pitch shift: two heads half a window apart, crossfaded, each new grain aligned ----
    psBuf[psWrite] = s;
    float newPhase = frac1(psPhase + psStep);
    for (int h = 0; h < 2; h++) {
      float pOld = frac1(psPhase + 0.5f * h), pNew = frac1(newPhase + 0.5f * h);
      if (fabsf(pNew - pOld) > 0.5f) psAlign(h, newPhase);   // this head jumps (its gain is 0): start in phase
    }
    psPhase = newPhase;
    float phB = frac1(psPhase + 0.5f);
    float gA = 0.5f - 0.5f * cosf(2.0f * PI * psPhase);
    float gB = 1.0f - gA;
    float shifted = psRead(PS_BASE + psPhase * SHIFT_WINDOW + psOff[0]) * gA
                  + psRead(PS_BASE + phB * SHIFT_WINDOW + psOff[1]) * gB;
    psWrite = (psWrite + 1) & (PS_LEN - 1);
    psMix += (mixTarget - psMix) * 0.0005f;                  // fades in and out over about 50 ms
    s = s + (shifted - s) * psMix;
 
    // ---- Delay: the time glides when changed, like tape ----
    if (dlyLen > 4) {
      dlySamplesS += (dlyTarget - dlySamplesS) * 0.0002f;
      dlyBuf[dlyWrite] = (int16_t)(fminf(fmaxf(s, -1.0f), 1.0f) * 32767.0f);
      if (dlySamplesS > 1.0f) {
        float rp = dlyWrite - dlySamplesS;
        while (rp < 0) rp += dlyLen;
        uint32_t i0 = (uint32_t)rp; float fr = rp - i0;
        uint32_t i1 = i0 + 1; if (i1 >= dlyLen) i1 = 0;
        s = (dlyBuf[i0] + (dlyBuf[i1] - dlyBuf[i0]) * fr) / 32767.0f;
      }
      if (++dlyWrite >= dlyLen) dlyWrite = 0;
    }
 
    int32_t o = (int32_t)(s * VOLUME * 2147483000.0f);
    txBuf[2 * i] = o;                                        // left
    txBuf[2 * i + 1] = o;                                    // right, same signal
  }
 
  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);
 
  // ---- LED: blinking while a touch point is held, otherwise on while it sings ----
  if (millis() < resetFlashUntil) digitalWrite(PIN_LED, (millis() / 50) % 2 ? HIGH : LOW);
  else if (job == JOB_RESET) digitalWrite(PIN_LED, HIGH);
  else if (job == JOB_PITCH) digitalWrite(PIN_LED, (millis() / 300) % 2 ? HIGH : LOW);
  else if (job == JOB_DELAY) digitalWrite(PIN_LED, (millis() / 100) % 2 ? HIGH : LOW);
  else                       digitalWrite(PIN_LED, ratio > 0.3f ? HIGH : LOW);
 
  // ---- Monitor, four times per second (115200 baud) ----
  if (++blockCount % 86 == 0) {
    Serial.printf("%s | gain %.3f | level %.2f / %.2f | fatigue %.2f | pot: %s%s | filter %4.0f Hz | pitch %.3f | delay %4.0f ms\n",
                  tired ? "RESTING" : "growing", gain, amp, AMP_MAX, fatigue,
                  JOB_NAMES[job], armed ? "" : " (turn to pick up)", lpfHigh, pitch, dlySamplesS * 1000.0f / SR);
    if (DEBUG_TOUCH)
      Serial.printf("   touch D4: %4.0f / %4.0f -> %4.0f %-7s | D32: %4.0f / %4.0f -> %4.0f %s\n",
                    touchLast[0], touchBase[0], touchBase[0] * TOUCH_RATIO, touchDown[0] ? "TOUCHED" : "-",
                    touchLast[1], touchBase[1], touchBase[1] * TOUCH_RATIO, touchDown[1] ? "TOUCHED" : "-");
  }
}
 