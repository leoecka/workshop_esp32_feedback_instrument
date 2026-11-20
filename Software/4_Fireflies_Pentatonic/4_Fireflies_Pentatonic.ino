// =====================================================================
//  FIREFLIES PENTATONIC – coupled oscillators through the air
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  Every board is an oscillator: a counter runs from 0 to 1. At 1 it plays
//  a note and starts again. While the microphone hears sound above the room
//  noise, the counter runs faster, the more so the further it already is.
//  This pulls several boards into a shared rhythm. A board is deaf only
//  while its own note is sounding, so it never pushes itself.
//  Each note is chosen from five notes of A minor pentatonic.
//
//  The pot moves through one landscape (fully down = 0):
//    register   a window of five pentatonic notes, lowest note A 55 Hz
//               at the bottom, up to A 440 Hz at the top (TRANSPOSE = -12)
//    character  bottom: dark pads, half a cycle long, that swell and fade
//               top:    short, bright, percussive notes (about 8 % of a cycle),
//               so percussive boards listen almost all the time
//    tempo      bottom third: period x2 (6 s), middle: x1 (3 s),
//               top third: x1/2 (1.5 s)
//  Boards set to different positions play in 1:2:4 relations, so their
//  rhythms and harmonies can be watched together.
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
#define PIN_LED   2    // blue LED on the DevKit, flashes with each note

// ---------- Things to play with ----------
const float PERIOD_SEC     = 3.0f;   // base period in the middle position
const float PERIOD_LOW     = 2.0f;   // multiplier at the bottom (slower)
const float PERIOD_HIGH    = 0.5f;   // multiplier at the top (faster)
const bool  QUANTIZE_TEMPO = false;   // true: three steps (x2, x1, x1/2), false: continuous
const float COUPLING    = 6.0f;      // how strongly heard onsets speed it up (0 = no coupling)
const float MIC_THRESH  = 0.002f;    // minimum level above the room noise that counts
const float NOTE_PAD    = 0.5f;      // note length at the bottom, as a fraction of the period
const float NOTE_PERC   = 0.08f;     // note length at the top, as a fraction of the period
const float LATENCY_SEC = 0.03f;     // own sound still travelling through buffers and air
const float VOLUME      = 0.45f;
const bool  POT_INVERT  = true;      // on this board, fully down reads the highest value
const int   MIC_SLOT    = 0;         // 0 = left. If the mic level stays dead in the monitor, try 1

// ---------- A minor pentatonic, A2 (110 Hz) up to G6, shifted by TRANSPOSE ----------
const int TRANSPOSE = -12;                // semitones: -12 = one octave down, 0 = original
const int NOTES[] = { 45, 48, 50, 52, 55,  57, 60, 62, 64, 67,
                      69, 72, 74, 76, 79,  81, 84, 86, 88, 91 };   // MIDI numbers
const int NOTE_N  = sizeof(NOTES) / sizeof(NOTES[0]);
const int WINDOW  = 5;                    // five notes to choose from
const int LOWEST_START  = 0;              // window starts at A2 = 110 Hz
const int HIGHEST_START = 15;             // window starts at A5 = 880 Hz
const char *NAMES[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

// ---------- Audio ----------
const int SR = 44100;
const int N  = 128;                       // frames per block, about 2.9 ms
int32_t rxBuf[N * 2], txBuf[N * 2];
i2s_chan_handle_t txChan, rxChan;

// ---------- Voices: sawtooth through a lowpass, they may overlap ----------
struct Voice {
  bool on;
  uint32_t n, attN, decN;        // samples since start, attack length, release length
  int curve;                     // release shape: 2 = soft (pads) .. 4 = steep (percussion)
  float pa, pb, inc, detune, env, fcBase, fcSweep, q, ic1, ic2;
};
const int VOICES = 4;
Voice vo[VOICES];

// ---------- State ----------
float phase = 0, potS = 0.5f;
int   windowStart = 0, lastNote = -1;
float hpX = 0, hpY = 0, micLvl = 0, micFloor = 0.001f;
bool  ownSounding = false;                // is any of its own notes playing right now
uint32_t sinceOwnEnd = 1000000;           // samples since its own last note ended
uint32_t ledOffAt = 0, blockCount = 0;

static inline float mixf(float a, float b, float x) { return a + (b - a) * x; }
static inline float mtof(int m) { return 440.0f * powf(2.0f, (m - 69) / 12.0f); }

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

// ---------- Play one note; its character comes from the pot ----------
void trigger(float perc, float periodSec) {   // perc: 0 = pad, 1 = percussive
  // pick a free voice, or steal the quietest one
  int v = 0; float quiet = 2.0f;
  for (int i = 0; i < VOICES; i++) {
    if (!vo[i].on) { v = i; break; }
    if (vo[i].env < quiet) { quiet = vo[i].env; v = i; }
  }
  // one of the five notes in the current window, never the same twice in a row
  int idx;
  do { idx = windowStart + (int)(esp_random() % WINDOW); } while (idx == lastNote && WINDOW > 1);
  lastNote = idx;

  float pad = 1.0f - perc;
  Voice &x = vo[v];
  x.on = true; x.n = 0; x.env = 0; x.pa = 0; x.pb = 0.5f;
  x.inc     = mtof(NOTES[idx] + TRANSPOSE) / SR;
  x.detune  = 1.0f + pad * 0.006f;                                // pads: two slightly detuned saws
  // note length relative to the cycle: half a period for pads, shorter towards percussion
  float lenSec = periodSec * NOTE_PAD * powf(NOTE_PERC / NOTE_PAD, perc);
  float attSec = fmaxf(0.003f, lenSec * 0.4f * pad * pad);        // pads swell, percussion hits
  x.attN    = (uint32_t)(attSec * SR);
  x.decN    = (uint32_t)(fmaxf(lenSec - attSec, 0.01f) * SR);
  x.curve   = 2 + (int)roundf(2.0f * perc);
  x.fcBase  = mtof(NOTES[idx] + TRANSPOSE) * 1.2f;
  x.fcSweep = mixf(800.0f, 3500.0f, perc);                        // pads open gently, percussion bright
  x.q       = mixf(1.0f, 2.5f, perc);

  digitalWrite(PIN_LED, HIGH);
  ledOffAt = millis() + 50;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);
  phase = (esp_random() % 1000) / 1000.0f;   // every board starts somewhere else in the cycle
  setupI2S();
  Serial.println("Fireflies pentatonic ready.");
}

void loop() {
  // reading one block of microphone data also sets the timing
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);

  // ---- Pot: 0 = fully down ----
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.05f;

  // register: slide the five-note window, with a little hysteresis against flicker
  float pos = potS * (HIGHEST_START - LOWEST_START) + LOWEST_START;
  if (fabsf(pos - windowStart) > 0.6f) windowStart = (int)roundf(pos);

  // tempo: x2 at the bottom, x1 in the middle, x1/2 at the top
  float mult;
  if (QUANTIZE_TEMPO) mult = potS < 0.333f ? PERIOD_LOW : (potS < 0.667f ? 1.0f : PERIOD_HIGH);
  else                mult = PERIOD_LOW * powf(PERIOD_HIGH / PERIOD_LOW, potS);
  const float dPhase = 1.0f / (PERIOD_SEC * mult * SR);

  // lowpass of every voice, once per block, following its envelope
  float a1[VOICES], a2[VOICES], a3[VOICES];
  for (int v = 0; v < VOICES; v++) {
    float fc = vo[v].fcBase + vo[v].fcSweep * vo[v].env * vo[v].env;
    float g = tanf(PI * fminf(fc, 9000.0f) / SR), k = 1.0f / fmaxf(vo[v].q, 0.5f);
    a1[v] = 1.0f / (1.0f + g * (g + k)); a2[v] = g * a1[v]; a3[v] = g * a2[v];
  }

  for (int i = 0; i < N; i++) {
    // ---- Mic: 24 bit in the top of a 32 bit word, remove DC ----
    float x = (rxBuf[2 * i + MIC_SLOT] >> 8) / 8388608.0f;
    float y = x - hpX + 0.995f * hpY;  hpX = x; hpY = y;
    float r = fabsf(y);
    micLvl += (r - micLvl) * (r > micLvl ? 0.02f : 0.0005f);

    // ---- Hearing: the level above the room noise, deaf while its own note sounds ----
    bool deaf = ownSounding || sinceOwnEnd < (uint32_t)(LATENCY_SEC * SR);
    if (!deaf && micLvl < micFloor * 2.0f) micFloor += (micLvl - micFloor) * 0.00002f;  // learn room noise
    float drive = 0;
    if (!deaf) {
      float over = micLvl - (micFloor * 2.0f + MIC_THRESH);
      if (over > 0) drive = fminf(over / MIC_THRESH, 1.0f);
    }

    // ---- Firefly: heard onsets speed up the counter, the more the later it is ----
    phase += dPhase * (1.0f + COUPLING * drive * phase);
    if (phase >= 1.0f) { phase = 0; trigger(potS, PERIOD_SEC * mult); }

    // ---- Voices ----
    float mix = 0;
    bool  sounding = false;
    for (int v = 0; v < VOICES; v++) {
      Voice &s = vo[v];
      if (!s.on) continue;
      // envelope with a real end: rise over attN, then fall to exactly zero over decN
      if (s.n < s.attN) s.env = (float)s.n / s.attN;
      else {
        float u = (float)(s.n - s.attN) / s.decN;
        if (u >= 1.0f) { s.on = false; continue; }
        float rest = 1.0f - u, e = rest;
        for (int c = 1; c < s.curve; c++) e *= rest;
        s.env = e;
      }
      s.n++;
      sounding = true;
      float ib = s.inc * s.detune;
      float saw = (2.0f * s.pa - 1.0f - polyblep(s.pa, s.inc)) + (2.0f * s.pb - 1.0f - polyblep(s.pb, ib));
      s.pa += s.inc; if (s.pa >= 1.0f) s.pa -= 1.0f;
      s.pb += ib;    if (s.pb >= 1.0f) s.pb -= 1.0f;
      float v3 = 0.5f * saw - s.ic2;
      float v1 = a1[v] * s.ic1 + a2[v] * v3;
      float v2 = s.ic2 + a2[v] * s.ic1 + a3[v] * v3;
      s.ic1 = 2.0f * v1 - s.ic1; s.ic2 = 2.0f * v2 - s.ic2;
      mix += v2 * s.env;
    }
    if (ownSounding && !sounding) sinceOwnEnd = 0;
    ownSounding = sounding;
    if (sinceOwnEnd < 0xFFFFFF) sinceOwnEnd++;
    float out = tanhf(mix * 0.7f) * VOLUME;                  // soft limit when pads overlap
    int32_t o = (int32_t)(out * 2147483000.0f);
    txBuf[2 * i] = o;                                        // left
    txBuf[2 * i + 1] = o;                                    // right, same signal
  }

  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);
  if (ledOffAt && millis() > ledOffAt) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }

  // ---- Monitor, twice per second (115200 baud) ----
  if (++blockCount % 172 == 0) {
    int lo = NOTES[windowStart] + TRANSPOSE, hi = NOTES[windowStart + WINDOW - 1] + TRANSPOSE;
    Serial.printf("pot %.2f | notes %s%d..%s%d (%.0f..%.0f Hz) | %s | period %.1f s | cycle %.2f | mic %.4f noise %.4f %s\n",
                  potS, NAMES[lo % 12], lo / 12 - 1, NAMES[hi % 12], hi / 12 - 1, mtof(lo), mtof(hi),
                  potS < 0.33f ? "pad" : (potS < 0.67f ? "between" : "percussive"),
                  PERIOD_SEC * mult, phase, micLvl, micFloor, ownSounding ? "(deaf)" : "");
  }
}
