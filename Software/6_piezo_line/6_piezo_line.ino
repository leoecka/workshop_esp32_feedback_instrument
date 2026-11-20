
// =====================================================================
//  PIEZO / LINE PASSTHROUGH
//  Feedback workshop board v2 · Leon Eckard 2026
//
//  A1 (GPIO 36) -> median -> oversampling -> remove DC -> highpass 80 Hz
//  -> optional notches -> spectral noise reduction -> optional gate -> pot -> I2S
//
//  INPUT – works best with a small circuit in front of A1:
//    Piezo:   an impedance converter (JFET buffer, e.g. the Modal States piezo
//             preamp). A piezo needs a very high input impedance; the ESP's
//             analog input would load it, so bass and level are lost.
//    Line in: shift the signal to the middle of the 3.3 V range (about 1.65 V),
//             e.g. a capacitor (1 to 10 µF) in series, then 100 kΩ to 3.3 V and
//             100 kΩ to GND at A1. The ESP only measures between 0 and 3.3 V;
//             a signal swinging around 0 V would lose its lower half.
//    In both cases: 10 nF from A1 to GND, close to the ESP, against aliasing.
//    Use only one channel of a stereo source, do not connect left and right directly.
//
//  With DIAG = true the serial monitor shows twice per second:
//    DC     resting level at the input (raw value and roughly in volts)
//    AC     signal level without DC, in dBFS
//    low    samples in the dead zone of the ADC (below about 0.15 V) -> the signal is clipped at the bottom
//    high   samples at the top end -> the signal is clipped at the top
//    fill   fill level of the buffer, ratio = smooth clock compensation
//    tones  the three loudest frequencies in the signal, to find whines and hum
//
//  Sample rate 50 kHz: at 44.1 and 48 kHz the clock division in the ESP32 produces a whine.
//  Low latency: blocks of 64, buffer about 3 ms. In total about 9 ms with, about 6 ms without noise reduction.
//  Arduino-ESP32 3.x (ESP-IDF 5), board "ESP32 Dev Module".
// =====================================================================
 
#include <Arduino.h>
#include <driver/i2s_std.h>
#include <esp_adc/adc_continuous.h>
#include <math.h>
#include "esp_dsp.h"
 
// ---------- Pins ----------
#define PIN_BCLK  26
#define PIN_WS    25
#define PIN_DOUT  22               // to AMP and LINE OUT
#define CH_IN     ADC_CHANNEL_0    // A1 = GPIO 36
#define CH_POT    ADC_CHANNEL_6    // pot = GPIO 34
 
// ---------- Diagnostics ----------
// true: monitor with levels and frequency analysis. The analysis makes the output stall
// briefly twice per second, which you hear as a faint rhythm. Set to false for playing.
const bool DIAG = false;
 
// ---------- Input ----------
// DB_12: measures about 0.15 .. 2.45 V cleanly (safe, coarser)
// DB_6:  measures about 0.15 .. 1.75 V cleanly (finer) – only if DC is safely between about 0.5 and 1.3 V
const adc_atten_t IN_ATTEN = ADC_ATTEN_DB_12;
 
// ---------- Against the ADC's own noise ----------
const bool USE_MEDIAN = true;      // removes single spikes (median of 3 values)
const int  OVERSAMPLE = 4;         // the ADC measures this much faster, and this many values are averaged (1 = off)
 
// ---------- Sound ----------
const float MAX_VOLUME = 1.0f;     // volume with the pot fully up
const bool  POT_INVERT = true;     // fully down = quiet
// Against the whine at 44.1 kHz: interference tone of the ESP32 at about 3050 Hz and its overtones
const bool  USE_NOTCHES = false;   // not needed at 50 kHz
const float NOTCH_FREQS[] = { 3047.0f, 6105.0f, 9152.0f, 12199.0f };
const float NOTCH_DB   = -36.0f;   // how deep the cut is
const float NOTCH_Q    = 8.0f;     // how narrow (higher = narrower)
// Against mains hum (50 Hz): highpass
const bool  USE_HPF    = true;
const float HPF_HZ     = 80.0f;
 
// ---------- Gate against the remaining noise between hits ----------
// With look-ahead: the signal is delayed a little, so the gate is already open
// when a hit arrives. The sketch learns the threshold itself from the noise level.
const bool  USE_GATE        = false;    // usually not needed with the noise reduction
const float GATE_LOOKAHEAD_MS = 1.5f;   // how far the gate looks ahead
const float GATE_ABOVE_DB   = 16.0f;    // opens this many dB above the learned noise
const float GATE_HOLD_MS    = 120.0f;   // stays open at least this long
const float GATE_RELEASE_MS = 80.0f;    // fades out this softly
const float GATE_RANGE_DB   = -40.0f;   // this much quieter when closed (not fully silent, less noticeable)
 
// ---------- Spectral noise reduction ----------
// Splits the signal into frequency bands, learns how loud the noise is in each band,
// and lowers every band the more, the closer it is to the noise.
// Leave about 2 seconds of silence after switching on, then it keeps learning by itself.
// Meant for piezo hits with silence in between. On music it also removes reverb tails
// and can sound watery: then use NR_STRENGTH = 0 (no reduction, but the steadier sound).
const bool  USE_NR      = true;
const float NR_STRENGTH = 0.0f;    // how strongly noise is subtracted (1 = exactly the noise, more = more thorough)
const float NR_FLOOR_DB = -18.0f;  // lower at most this far (deeper = quieter, but more bubbling)
const float NR_LEARN_S  = 1.5f;    // silence needed at the start for learning
const float NR_RISE_S   = 5.0f;    // how slowly the learned noise may rise later
 
// ---------- Audio ----------
const int SR = 50000;              // output and ADC per channel. 50 kHz: no whine (at 44.1 and 48 kHz the ESP clock whines)
const int N  = 64;                 // samples per block: small = little latency (64 = 1.3 ms)
int32_t txBuf[N * 2];
i2s_chan_handle_t txChan;
adc_continuous_handle_t adcH;
 
// Ring buffer with smooth reading (length must be a power of two)
const int RING = 4096, MASK = RING - 1, TARGET = 160;   // buffer target: about 3 ms in stock
float ring[RING];
int   head = 0;
float readPos = 0;
float ratio = 1.0f;
bool  primed = false;
float lastX = 0;
 
// Pot
float potRawSum = 0; int potRawN = 0; float potVal = 0.0f; float volSmooth = 0.0f;
 
// Diagnostics
double dcSum = 0; uint32_t dcN = 0, clipLow = 0, clipHigh = 0;
double acSum = 0; uint32_t acN = 0; float acPeak = 0;
uint32_t blockCount = 0;
 
// Frequency analysis for the monitor: the three loudest tones in the signal
const int FFT_N = 4096;                       // about 0.08 s, about 12 Hz resolution
float fftIn[FFT_N];                           // signal after DC removal
__attribute__((aligned(16))) float fftBuf[FFT_N * 2];
float hann[FFT_N];
int fftPos = 0;
 
// Gate state
const int GATE_DLY_MAX = 512;
float gateDly[GATE_DLY_MAX];
int   gateDlyPos = 0, gateDlyN = 1;
float gateEnv = 0, gateNoise = 0.01f, gateGain = 1, gateFloor = 0;
float gateAtt = 0, gateRel = 0, gateEnvRel = 0, gateNoiseUp = 0, gateOpenFactor = 0;
uint32_t gateHold = 0, gateHoldN = 0;
bool gateOpen = false;
 
// Noise reduction state: frames of 4 blocks, advancing by one block (75 % overlap)
const int NR_FFT = 4 * N;            // with N = 64: 256 values, 3.8 ms latency
const int NR_BINS = NR_FFT / 2 + 1;
float nrIn[NR_FFT], nrOla[NR_FFT], nrWin[NR_FFT];
__attribute__((aligned(16))) float nrBuf[NR_FFT * 2];
float nrP[NR_BINS], nrNoise[NR_BINS], nrG[NR_BINS];
uint32_t nrFrames = 0, nrLearnFrames = 1;
float nrRise = 0, nrFloorPow = 0;
 
// Remove DC (10 Hz)
float dcX = 0, dcY = 0;
const float dcCoef = expf(-2.0f * PI * 10.0f / SR);
 
// Biquad filters: peaking (used as notch) and highpass
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void initPeaking(float f, float gainDB, float Q, float sr) {
    float A = powf(10.0f, gainDB / 40.0f), w0 = 2.0f * PI * f / sr;
    float alpha = sinf(w0) / (2.0f * Q), c = cosf(w0);
    float a0 = 1.0f + alpha / A;
    b0 = (1.0f + alpha * A) / a0; b1 = (-2.0f * c) / a0; b2 = (1.0f - alpha * A) / a0;
    a1 = (-2.0f * c) / a0;        a2 = (1.0f - alpha / A) / a0;
  }
  void initHighpass(float f, float Q, float sr) {
    float w0 = 2.0f * PI * f / sr, alpha = sinf(w0) / (2.0f * Q), c = cosf(w0);
    float a0 = 1.0f + alpha;
    b0 = ((1.0f + c) / 2.0f) / a0; b1 = (-(1.0f + c)) / a0; b2 = ((1.0f + c) / 2.0f) / a0;
    a1 = (-2.0f * c) / a0;         a2 = (1.0f - alpha) / a0;
  }
  inline float process(float x) {
    float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y;
  }
};
const int NOTCH_N = sizeof(NOTCH_FREQS) / sizeof(NOTCH_FREQS[0]);
Biquad notches[NOTCH_N];
Biquad hpf;
 
float fullScaleVolts() {   // only for the display, roughly
  switch (IN_ATTEN) {
    case ADC_ATTEN_DB_0:   return 0.95f;
    case ADC_ATTEN_DB_2_5: return 1.25f;
    case ADC_ATTEN_DB_6:   return 1.75f;
    default:               return 3.1f;
  }
}
 
void setupADC() {
  adc_continuous_handle_cfg_t hcfg = {};
  hcfg.max_store_buf_size = 16384;
  hcfg.conv_frame_size    = 512;     // small packets: the ADC delivers every 0.6 ms
  ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &adcH));
  static adc_digi_pattern_config_t pat[2];
  pat[0].atten = IN_ATTEN;        pat[0].channel = CH_IN;  pat[0].unit = ADC_UNIT_1; pat[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  pat[1].atten = ADC_ATTEN_DB_12; pat[1].channel = CH_POT; pat[1].unit = ADC_UNIT_1; pat[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  adc_continuous_config_t cfg = {};
  cfg.pattern_num    = 2;
  cfg.adc_pattern    = pat;
  cfg.sample_freq_hz = SR * 2 * OVERSAMPLE;   // both channels together, oversampled
  cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  cfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  ESP_ERROR_CHECK(adc_continuous_config(adcH, &cfg));
  ESP_ERROR_CHECK(adc_continuous_start(adcH));
}
 
void setupI2S() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);   // port 0 is used by the ADC
  chanCfg.dma_desc_num = 3; chanCfg.dma_frame_num = N; chanCfg.auto_clear = true;   // short output buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, &txChan, NULL));
  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SR),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)PIN_BCLK, .ws = (gpio_num_t)PIN_WS,
      .dout = (gpio_num_t)PIN_DOUT, .din = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txChan, &stdCfg));
  ESP_ERROR_CHECK(i2s_channel_enable(txChan));
}
 
static inline int available() { return (head - (int)readPos) & MASK; }
 
static inline float median3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}
 
void pullADC() {
  static uint8_t buf[1024];
  static float m0 = 0, m1 = 0, osAcc = 0;
  static int osCount = 0;
  uint32_t n = 0;
  while (adc_continuous_read(adcH, buf, sizeof(buf), &n, 0) == ESP_OK && n > 0) {
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= n; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *d = (adc_digi_output_data_t *)&buf[i];
      uint32_t ch = d->type1.channel, val = d->type1.data;
      if (ch == CH_IN) {
        dcSum += val; dcN++;
        if (val <= 10) clipLow++;
        if (val >= 4085) clipHigh++;
        float v = (val - 2048.0f) / 2048.0f;
        if (USE_MEDIAN) { float m = median3(m0, m1, v); m0 = m1; m1 = v; v = m; }
        osAcc += v;
        if (++osCount >= OVERSAMPLE) {
          float s = osAcc / OVERSAMPLE; osAcc = 0; osCount = 0;
          if (available() < RING - 2) { ring[head] = s; head = (head + 1) & MASK; }
        }
      } else if (ch == CH_POT) {
        potRawSum += val; potRawN++;
      }
    }
  }
  if (potRawN > 0) { potVal = potRawSum / potRawN / 4095.0f; potRawSum = 0; potRawN = 0; }
}
 
// ---- Noise reduction for one block of N values (in place) ----
void nrProcess(float *io) {
  // append the new block at the end
  memmove(nrIn, nrIn + N, (NR_FFT - N) * sizeof(float));
  memcpy(nrIn + NR_FFT - N, io, N * sizeof(float));
  for (int i = 0; i < NR_FFT; i++) { nrBuf[2 * i] = nrIn[i] * nrWin[i]; nrBuf[2 * i + 1] = 0; }
  dsps_fft2r_fc32(nrBuf, NR_FFT);
  dsps_bit_rev_fc32(nrBuf, NR_FFT);
 
  bool learning = nrFrames < nrLearnFrames;
  for (int k = 0; k < NR_BINS; k++) {
    float re = nrBuf[2 * k], im = nrBuf[2 * k + 1];
    float pw = re * re + im * im;
    nrP[k] = 0.6f * nrP[k] + 0.4f * pw;                                   // smoothed over time
    if (learning)               nrNoise[k] += (nrP[k] - nrNoise[k]) / (float)(nrFrames + 1);  // average of the silence
    else if (nrP[k] < 3.0f * nrNoise[k]) nrNoise[k] += (nrP[k] - nrNoise[k]) * 0.02f;  // quiet band: keep learning
    else                        nrNoise[k] += (nrP[k] - nrNoise[k]) * nrRise;          // sound in the band: only very slowly
    float gp = 1.0f - NR_STRENGTH * nrNoise[k] / (nrP[k] + 1e-12f);
    if (gp < nrFloorPow) gp = nrFloorPow;
    float g = sqrtf(gp);
    nrG[k] = 0.5f * nrG[k] + 0.5f * g;                                   // smoothed: less bubbling
  }
  nrFrames++;
  // apply the gain to both halves of the spectrum
  for (int k = 0; k < NR_BINS; k++) {
    nrBuf[2 * k] *= nrG[k]; nrBuf[2 * k + 1] *= nrG[k];
    if (k > 0 && k < NR_FFT / 2) {
      int m = NR_FFT - k;
      nrBuf[2 * m] *= nrG[k]; nrBuf[2 * m + 1] *= nrG[k];
    }
  }
  // back to a time signal: conjugate, FFT, real part
  for (int i = 0; i < NR_FFT; i++) nrBuf[2 * i + 1] = -nrBuf[2 * i + 1];
  dsps_fft2r_fc32(nrBuf, NR_FFT);
  dsps_bit_rev_fc32(nrBuf, NR_FFT);
  const float scale = 0.5f / NR_FFT;                                       // window applied twice, 75 % overlap
  for (int i = 0; i < NR_FFT; i++) nrOla[i] += nrBuf[2 * i] * nrWin[i] * scale;
  // output the finished block, move the rest forward
  memcpy(io, nrOla, N * sizeof(float));
  memmove(nrOla, nrOla + N, (NR_FFT - N) * sizeof(float));
  memset(nrOla + NR_FFT - N, 0, N * sizeof(float));
}
 
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NOTCH_N; i++) notches[i].initPeaking(NOTCH_FREQS[i], NOTCH_DB, NOTCH_Q, (float)SR);
  hpf.initHighpass(HPF_HZ, 0.707f, (float)SR);
  for (int i = 0; i < NR_FFT; i++) nrWin[i] = sqrtf(0.5f - 0.5f * cosf(2.0f * PI * i / NR_FFT));   // square-root Hann
  for (int k = 0; k < NR_BINS; k++) { nrP[k] = 0; nrNoise[k] = 0; nrG[k] = 1; }
  nrLearnFrames = (uint32_t)(NR_LEARN_S * SR / N);
  nrRise        = 1.0f - expf(-1.0f / (NR_RISE_S * SR / N));
  nrFloorPow    = powf(10.0f, NR_FLOOR_DB / 10.0f);
  gateDlyN       = constrain((int)(GATE_LOOKAHEAD_MS * 0.001f * SR), 1, GATE_DLY_MAX - 1);
  gateAtt        = 1.0f - expf(-1.0f / (0.0003f * SR));              // opens in about 0.3 ms
  gateRel        = 1.0f - expf(-1.0f / (GATE_RELEASE_MS * 0.001f * SR));
  gateEnvRel     = expf(-1.0f / (0.005f * SR));                      // envelope falls in about 5 ms
  gateNoiseUp    = 1.0f - expf(-1.0f / (10.0f * SR));                // noise estimate may rise only very slowly
  gateHoldN      = (uint32_t)(GATE_HOLD_MS * 0.001f * SR);
  gateFloor      = powf(10.0f, GATE_RANGE_DB / 20.0f);
  gateOpenFactor = powf(10.0f, GATE_ABOVE_DB / 20.0f);
  dsps_fft2r_init_fc32(NULL, FFT_N);
  for (int i = 0; i < FFT_N; i++) hann[i] = 0.5f - 0.5f * cosf(2.0f * PI * i / (FFT_N - 1));
  setupI2S();
  setupADC();
  Serial.printf("Passthrough started, input up to about %.2f V, notches %s, highpass %s, median %s, oversampling x%d, diagnostics %s\n",
                fullScaleVolts(), USE_NOTCHES ? "on" : "off", USE_HPF ? "on" : "off", USE_MEDIAN ? "on" : "off", OVERSAMPLE, DIAG ? "on" : "off");
  if (USE_NR) Serial.println("Noise reduction is learning: please keep quiet for about 2 seconds.");
}
 
void loop() {
  pullADC();
 
  // ---- Pot ----
  float p = POT_INVERT ? (1.0f - potVal) : potVal;
  if (p < 0.015f) p = 0.0f;
  volSmooth += (p - volSmooth) * 0.05f;
  float vol = volSmooth * MAX_VOLUME;
 
  // ---- Smooth clock compensation: reads a tiny bit faster or slower ----
  int fill = available();
  if (!primed && fill >= TARGET) primed = true;
  float err = (fill - TARGET) / (float)TARGET;
  float ratioTarget = 1.0f + fminf(fmaxf(err * 0.02f, -0.05f), 0.05f);   // up to 5 %, the ADC runs about 2 % faster than set
  ratio += (ratioTarget - ratio) * 0.01f;
 
  // ---- 1. Read in and pre-filter ----
  static float blk[N];
  for (int i = 0; i < N; i++) {
    float x = lastX;
    if (primed && available() >= 2) {
      int   i0 = (int)readPos;
      float fr = readPos - i0;
      x = ring[i0] + (ring[(i0 + 1) & MASK] - ring[i0]) * fr;
      readPos += ratio;
      if (readPos >= RING) readPos -= RING;
      lastX = x;
    }
 
    // remove DC
    float y = dcCoef * (dcY + x - dcX);
    dcX = x; dcY = y;
 
    if (USE_HPF) y = hpf.process(y);
    if (USE_NOTCHES) for (int n = 0; n < NOTCH_N; n++) y = notches[n].process(y);
    blk[i] = y;
  }
 
  // ---- 2. Spectral noise reduction ----
  if (USE_NR) nrProcess(blk);
 
  // ---- 3. Gate, volume, output ----
  for (int i = 0; i < N; i++) {
    float y = blk[i];
 
    // ---- Gate with look-ahead ----
    if (USE_GATE) {
      // envelope on the undelayed signal
      float a = fabsf(y);
      gateEnv = (a > gateEnv) ? a : a + gateEnvRel * (gateEnv - a);
      // learn the noise level: falls quickly, rises only very slowly
      if (gateEnv < gateNoise) gateNoise += (gateEnv - gateNoise) * 0.001f;
      else                     gateNoise += (gateEnv - gateNoise) * gateNoiseUp;
      float openT  = gateNoise * gateOpenFactor;
      float closeT = openT * 0.7f;                                  // close a bit lower, against flutter
      if (gateEnv > openT) { gateOpen = true; gateHold = gateHoldN; }
      else if (gateHold > 0) gateHold--;
      else if (gateEnv < closeT) gateOpen = false;
      float target = gateOpen ? 1.0f : gateFloor;
      gateGain += (target - gateGain) * (gateOpen ? gateAtt : gateRel);
      // output the delayed signal
      float delayed = gateDly[(gateDlyPos + GATE_DLY_MAX - gateDlyN) % GATE_DLY_MAX];
      gateDly[gateDlyPos] = y; gateDlyPos = (gateDlyPos + 1) % GATE_DLY_MAX;
      y = delayed * gateGain;
    }
 
    // diagnostics before the volume
    if (DIAG) {
      fftIn[fftPos] = y; fftPos = (fftPos + 1) % FFT_N;
      acSum += (double)y * y; acN++;
      if (fabsf(y) > acPeak) acPeak = fabsf(y);
    }
 
    y *= vol;
    if (y > 1.0f) y = 1.0f; else if (y < -1.0f) y = -1.0f;
    int32_t s = (int32_t)(y * 2147483000.0f);
    txBuf[2 * i] = s; txBuf[2 * i + 1] = s;                 // left and right, same signal
  }
 
  size_t written = 0;
  i2s_channel_write(txChan, txBuf, sizeof(txBuf), &written, portMAX_DELAY);
 
  // ---- Monitor twice per second (only with DIAG) ----
  static uint32_t lastPrint = 0;
  if (DIAG && millis() - lastPrint >= 500) {
    lastPrint = millis();
    float dcRaw = dcN ? dcSum / dcN : 0;
    float rms = acN ? sqrt(acSum / acN) : 0;
    Serial.printf("DC %4.0f (about %.2f V) | AC %6.1f dBFS, peak %6.1f dBFS | low %lu high %lu | fill %4d ratio %.5f | vol %.2f | noise %.1f dBFS gate %s\n",
                  dcRaw, dcRaw / 4095.0f * fullScaleVolts(),
                  20.0f * log10f(rms + 1e-9f), 20.0f * log10f(acPeak + 1e-9f),
                  (unsigned long)clipLow, (unsigned long)clipHigh, fill, ratio, vol,
                  20.0f * log10f(gateNoise + 1e-9f), gateOpen ? "open" : "closed");
    dcSum = 0; dcN = 0; clipLow = 0; clipHigh = 0; acSum = 0; acN = 0; acPeak = 0;
 
    // ---- the three loudest tones above 40 Hz ----
    for (int i = 0; i < FFT_N; i++) {
      fftBuf[2 * i] = fftIn[(fftPos + i) % FFT_N] * hann[i];
      fftBuf[2 * i + 1] = 0;
    }
    dsps_fft2r_fc32(fftBuf, FFT_N);
    dsps_bit_rev_fc32(fftBuf, FFT_N);
    const float binHz = (float)SR / FFT_N;
    int   pk[3] = { -1, -1, -1 };
    float pv[3] = { 0, 0, 0 };
    for (int k = (int)(40 / binHz) + 1; k < FFT_N / 2 - 1; k++) {
      float m  = fftBuf[2 * k] * fftBuf[2 * k] + fftBuf[2 * k + 1] * fftBuf[2 * k + 1];
      float ml = fftBuf[2 * k - 2] * fftBuf[2 * k - 2] + fftBuf[2 * k - 1] * fftBuf[2 * k - 1];
      float mr = fftBuf[2 * k + 2] * fftBuf[2 * k + 2] + fftBuf[2 * k + 3] * fftBuf[2 * k + 3];
      if (m < ml || m < mr) continue;                       // only local peaks
      for (int s = 0; s < 3; s++) if (m > pv[s]) {
        for (int u = 2; u > s; u--) { pv[u] = pv[u - 1]; pk[u] = pk[u - 1]; }
        pv[s] = m; pk[s] = k; break;
      }
    }
    float norm = (FFT_N / 4.0f) * (FFT_N / 4.0f);          // Hann window: a sine with amplitude 1 ~ 0 dB
    Serial.printf("   loudest tones: %6.0f Hz (%5.1f dB)  %6.0f Hz (%5.1f dB)  %6.0f Hz (%5.1f dB)\n",
                  pk[0] * binHz, 10 * log10f(pv[0] / norm + 1e-12f),
                  pk[1] * binHz, 10 * log10f(pv[1] / norm + 1e-12f),
                  pk[2] * binHz, 10 * log10f(pv[2] / norm + 1e-12f));
  }
}
 