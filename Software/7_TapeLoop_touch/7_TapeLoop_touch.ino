// =====================================================================
//  TAPE LOOP – sound on sound through the air, with varispeed
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  The loop buffer behaves like a tape loop: a head runs over it at the
//  playback rate. It reads in between the samples, and every place it
//  passes is multiplied once by the MULTIPLIER and, while recording,
//  gets the microphone added on top.
//    rate 2   -> an octave higher, the loop takes half as long
//    rate 0.5 -> an octave lower, the loop takes twice as long
//  Record at 0.5 and go back to 1: the recording sounds an octave higher.
//  The fade per pass is the same at every rate.
//
//  POT          MULTIPLIER 0..1 (this is also the volume: 0 erases the loop within one pass)
//  hold D4      record (the LED lights)
//  hold D32     the pot sets the PLAYBACK RATE 0.5..2 (middle = 1, the LED blinks)
//  After changing what the pot does, a value only follows the pot once you
//  turn across it (pickup), so nothing jumps.
//
//  Arduino-ESP32 3.x (ESP-IDF 5), board "ESP32 Dev Module".
// =====================================================================

#include <Arduino.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <math.h>

// ---------- Pins of board v2 ----------
#define PIN_BCLK   26
#define PIN_WS     25
#define PIN_DOUT   22   // to AMP and LINE OUT
#define PIN_MIC    35   // data from the INMP441
#define PIN_POT    34
#define PIN_LED    2
#define PIN_RATE   32   // touch T9: hold to set the playback rate
#define PIN_REC    4    // touch T0: hold to record

// ---------- Things to play with ----------
const float RATE_MIN    = 0.5f;   // playback rate with the pot fully down (while D32 is held)
const float RATE_MAX    = 2.0f;   // playback rate with the pot fully up
const float OUT_GAIN    = 0.8f;   // fixed output level
const float MIC_GAIN    = 8.0f;   // how much the mic is amplified before recording
const bool  POT_INVERT  = true;   // on this board, fully down reads the highest value
const float TOUCH_RATIO = 0.85f;  // touched below 85 % of the resting value
const bool  DEBUG_TOUCH = false;  // show touch readings in the serial monitor
const float PICKUP      = 0.03f;
const size_t MEM_RESERVE = 40 * 1024;   // memory left free for the rest of the program

// ---------- Audio ----------
const int SR = 44100;
const int N  = 128;
int32_t rxBuf[N * 2], txBuf[N * 2];
i2s_chan_handle_t txChan, rxChan;

// ---------- Tape, made of several pieces of memory ----------
const int CHUNK_SAMPLES = 8192;            // 16 KB per piece
const int MAX_CHUNKS    = 64;
int16_t *chunks[MAX_CHUNKS];
int   numChunks = 0;
uint32_t tapeLen = 0;

static inline int16_t &cellAt(uint32_t i) { return chunks[i / CHUNK_SAMPLES][i % CHUNK_SAMPLES]; }

// ---------- Head ----------
uint32_t head = 0;      // the cell the head is over
float    frac = 0;      // how far between this cell and the next, 0..1
float    inAcc = 0;     // mic samples collected since the head last left a cell
int      inN = 0;

// ---------- Pot with two jobs ----------
float mult = 0.9f, ratePot = 0.5f, rate = 1.0f, rateS = 1.0f, potS = 0;
bool  potOnRate = false, armed = true;

// ---------- Touch ----------
const int TOUCH_PINS[2] = { PIN_RATE, PIN_REC };
float touchBase[2] = { 0, 0 }, touchLast[2] = { 0, 0 };
bool  touchDown[2] = { false, false };
uint8_t touchCount[2] = { 0, 0 };

// ---------- State ----------
float hpX = 0, hpY = 0, recGain = 0, micLvl = 0, tapeLvl = 0;
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

// soft limit so that layers never clip hard
static inline float softLimit(float x) {
  float a = fabsf(x);
  if (a <= 0.8f) return x;
  float y = 0.8f + 0.2f * tanhf((a - 0.8f) / 0.2f);
  return x < 0 ? -y : y;
}

static inline float rateFromPot(float p) { return RATE_MIN * powf(RATE_MAX / RATE_MIN, p); }   // exponential, middle = 1

void allocateTape() {
  while (numChunks < MAX_CHUNKS &&
         heap_caps_get_free_size(MALLOC_CAP_8BIT) > MEM_RESERVE + CHUNK_SAMPLES * sizeof(int16_t)) {
    int16_t *c = (int16_t *)heap_caps_malloc(CHUNK_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!c) break;
    memset(c, 0, CHUNK_SAMPLES * sizeof(int16_t));
    chunks[numChunks++] = c;
  }
  tapeLen = (uint32_t)numChunks * CHUNK_SAMPLES;
}

void setupI2S() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = N;
  chanCfg.auto_clear    = true;
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

  allocateTape();
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS = p; mult = p;                              // the pot starts on the multiplier

  setupI2S();
  Serial.printf("Tape loop ready. Tape length %.2f s at rate 1 (%d pieces), free memory left %u bytes\n",
                (float)tapeLen / SR, numChunks, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
  Serial.println("Pot = multiplier. Hold D4 to record, hold D32 and turn the pot to set the playback rate.");
}

void loop() {
  size_t got = 0;
  i2s_channel_read(rxChan, rxBuf, sizeof(rxBuf), &got, portMAX_DELAY);
  if (tapeLen < 2) return;

  // ---- Touch, about every 15 ms ----
  if (blockCount % 5 == 0) { readTouch(0); readTouch(1); }
  bool recording = touchDown[1];

  // ---- Pot: multiplier, or the playback rate while D32 is held ----
  bool wantRate = touchDown[0];
  if (wantRate != potOnRate) { potOnRate = wantRate; armed = false; }   // pot changes its job: pick up first
  float p = analogRead(PIN_POT) / 4095.0f;
  if (POT_INVERT) p = 1.0f - p;
  potS += (p - potS) * 0.08f;
  float stored = potOnRate ? ratePot : mult;
  if (!armed && fabsf(potS - stored) < PICKUP) armed = true;
  if (armed) { if (potOnRate) ratePot = potS; else mult = potS; }
  rate = rateFromPot(ratePot);

  // ---- LED: on while recording, blinking while D32 is held ----
  if (recording)      digitalWrite(PIN_LED, HIGH);
  else if (potOnRate) digitalWrite(PIN_LED, (millis() / 120) % 2 ? HIGH : LOW);
  else                digitalWrite(PIN_LED, LOW);

  const float recStep = 1.0f / (0.005f * SR);   // record gain fades in and out over 5 ms, no clicks
  float rateStep = (rate - rateS) / N;          // glide the rate across the block, no zipper noise

  for (int i = 0; i < N; i++) {
    // ---- Mic: remove DC, amplify, soft limit ----
    float x = (rxBuf[2 * i] >> 8) / 8388608.0f;
    float y = x - hpX + 0.995f * hpY;  hpX = x; hpY = y;
    float in = tanhf(y * MIC_GAIN);
    micLvl += (fabsf(in) - micLvl) * 0.0005f;

    if (recording) { recGain += recStep; if (recGain > 1.0f) recGain = 1.0f; }
    else           { recGain -= recStep; if (recGain < 0.0f) recGain = 0.0f; }

    // ---- Read between two cells ----
    uint32_t nextCell = head + 1; if (nextCell >= tapeLen) nextCell = 0;
    float a = cellAt(head) / 32767.0f, b = cellAt(nextCell) / 32767.0f;
    float out = (a + (b - a) * frac) * OUT_GAIN;
    tapeLvl += (fabsf(a) - tapeLvl) * 0.0005f;

    // ---- Move the head; every cell it leaves is faded once and recorded on ----
    inAcc += in; inN++;
    rateS += rateStep;
    frac += rateS;
    if (frac >= 1.0f) {
      float newIn = inAcc / inN;                // average of the mic since the last cell: less aliasing when slow
      while (frac >= 1.0f) {
        frac -= 1.0f;
        int16_t &cell = cellAt(head);
        float nw = softLimit(cell / 32767.0f * mult + newIn * recGain);
        cell = (int16_t)(nw * 32767.0f);
        if (++head >= tapeLen) head = 0;
      }
      inAcc = 0; inN = 0;
    }

    if (out > 1.0f) out = 1.0f; else if (out < -1.0f) out = -1.0f;
    int32_t s = (int32_t)(out * 2147483000.0f);
    txBuf[2 * i] = s;
    txBuf[2 * i + 1] = s;
  }

  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);

  // ---- Monitor, twice per second ----
  if (++blockCount % 172 == 0) {
    Serial.printf("%s | pot: %s%s | multiplier %.3f | rate %.2f (loop %.2f s) | tape level %.3f | mic %.3f\n",
                  recording ? "REC" : "   ", potOnRate ? "RATE" : "MULTIPLIER", armed ? "" : " (turn to pick up)",
                  mult, rateS, (float)tapeLen / SR / rateS, tapeLvl, micLvl);
    if (DEBUG_TOUCH)
      Serial.printf("   touch D32: %4.0f / %4.0f -> %4.0f %-7s | D4: %4.0f / %4.0f -> %4.0f %s\n",
                    touchLast[0], touchBase[0], touchBase[0] * TOUCH_RATIO, touchDown[0] ? "TOUCHED" : "-",
                    touchLast[1], touchBase[1], touchBase[1] * TOUCH_RATIO, touchDown[1] ? "TOUCHED" : "-");
  }
}