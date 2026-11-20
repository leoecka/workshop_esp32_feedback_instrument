# Workshop on Feedback Instrument with Embedded Systems

An acoustic and digital feedback instrument based on an **ESP32 DevKit**, developed for the workshop on DIY feedback instruments. The system closes an audio loop through the physical space via microphone, digital signal processing (DSP), and speaker/line-out—complemented by a potentiometer and capacitive touch contacts on the ESP32.

---

## 1. Hardware & Assembly

The PCB connects a **30-pin ESP32 DevKit** to a digital I2S microphone, an audio output stage, a potentiometer, and optional analog inputs.

### Components & Connectors
* **Microcontroller:** ESP32 DevKit (30-pin version; align the antenna end over the keep-out zone on the PCB).
* **Microphone (`J2`):** **INMP441** I2S MEMS microphone (mounted directly or via Dupont wires; `L/R` is tied to `GND` for the left channel).
* **Audio Output (`J1` / `J3`):** Both headers are wired in parallel to the same I2S lines and can be populated individually or simultaneously:
  * **`J1` (AMP):** **MAX98357A** Class-D mono amplifier for driving a speaker directly.
  * **`J3` (LINE OUT):** **PCM5102A** stereo DAC for line-out / headphones.
    > **Important:** On the back of the PCM5102A module, the solder jumpers must be set to `FLT = L`, `DEMP = L`, `XSMT = H`, and `FMT = L`—otherwise the module will remain muted.
* **Controls:**
  * **`RV1`:** 10k linear potentiometer (9 mm, e.g., Alps RK09K, Bourns PTV09, or Alpha).
  * **Capacitive Touch Pins:** Solder pads **`D4`** (`GPIO 4`) and **`D32`** (`GPIO 32`) directly on the ESP32 board act as touch-sensitive buttons in several sketches.
* **Expansion (`J4` – Piezo):** 4-pin header (`3V3`, `GND`, `A1`, `A2`) with 1k protection resistors—e.g., for a piezo preamp, line-in, or additional potentiometers.

### On-Board Jumpers
* **`JP1` (AMP Left Only):** Bridged = amplifier plays left channel only | Open = mix of left and right channels.
* **`JP2` (AMP Gain):** Bridge `1–2` = 12 dB | Open = 9 dB | Bridge `2–3` = 6 dB.

### Pinout (ESP32 DevKit)
| Function | GPIO | Pin / Notes |
| :--- | :--- | :--- |
| **I2S BCLK** | `GPIO 26` | Shared Bit-Clock (Mic & Out) |
| **I2S WS / LRC** | `GPIO 25` | Shared Word-Select (Mic & Out) |
| **I2S DOUT** | `GPIO 22` | Audio Data to AMP / Line-Out |
| **I2S MIC IN** | `GPIO 35` | Audio Data from INMP441 |
| **Potentiometer** | `GPIO 34` | Analog Input (ADC1) |
| **Touch 1** | `GPIO 4` | Solder pad `D4` on the DevKit |
| **Touch 2** | `GPIO 32` | Solder pad `D32` on the DevKit |
| **Piezo A1 / A2** | `GPIO 36` / `39` | Analog expansion inputs |
| **Status LED** | `GPIO 2` | Onboard blue LED of the ESP32 |

---

## 2. Software Setup & Installation

All sketches are written for the **Arduino IDE** and require **Arduino-ESP32 Core version 3.x** (based on ESP-IDF 5).

### Step 1: Install Arduino IDE & ESP32 Board Package
1. Download and install the latest **Arduino IDE (2.x)**.
2. Open **Preferences / Settings** (*File → Preferences* on Windows/Linux or *Arduino IDE → Settings* on macOS).
3. Under **Additional Boards Manager URLs**, add the following URL:
   ```text
   [https://espressif.github.io/arduino-esp32/package_esp32_index.json](https://espressif.github.io/arduino-esp32/package_esp32_index.json)
   ```
4. Open the **Boards Manager** on the left sidebar, search for **`esp32`**, and install **`esp32` by Espressif Systems** (version **3.x** or newer).

### Step 2: Select Board
Go to **Tools → Board → esp32** and select **`ESP32 Dev Module`**. *(Default settings for Upload Speed, Flash Frequency, etc. can be left unchanged.)*

### Step 3: USB Drivers & Port Selection by OS
Connect the ESP32 to your computer using a **USB data cable** (*note: some USB cables are power-only and will not transmit data*). Select the port under **Tools → Port**:

* **Windows:**
  * The port appears as **`COMx`** (e.g., `COM3`, `COM4`).
  * **Tip:** Open the **Windows Device Manager** under *Ports (COM & LPT)* to see which COM port number appears when plugging in the board.
  * **Drivers:** If no COM port shows up or a yellow warning icon appears, install the USB-to-UART driver matching the chip on your DevKit: either **Silicon Labs CP210x** (square chip) or **WCH CH340 / CH341** (rectangular chip).
* **macOS:**
  * The port appears as `/dev/cu.usbserial-...` or `/dev/cu.SLAB_USBtoUART` (on recent macOS versions, drivers for CP210x and CH340 are usually built in; if not, install the corresponding driver from Silicon Labs or WCH).
* **Linux:**
  * The port appears as `/dev/ttyUSB0` (or `ttyACM0`).
  * If you encounter a *Permission denied* error during upload, add your user to the `dialout` group and log out/in again:
```bash
sudo usermod -a -G dialout $USER
```

### Step 4: Upload & Serial Monitor
1. Open any sketch from this repository and click **Upload**. *(If the upload hangs at `"Connecting..."`, press and hold the **BOOT** button on the ESP32 until flashing begins.)*
2. Open the **Serial Monitor** (**Tools → Serial Monitor**) and set the baud rate in the bottom-right corner to **`115200`**. It displays live telemetry including audio levels, filter cutoff frequencies, touch states, and system status.

---

## 3. Included Sketches (Sound Engines)

* **`FB_Pure`:** Acoustic feedback governed by a "Modal States" behavior. System gain ramps up until the loop begins to sing; as resonance builds, the system "fatigues," drops down, rests, and grows again. The potentiometer controls the low-pass filter cutoff (`60–2000 Hz`).
* **`Self-FM`:** A sawtooth oscillator modulates its own pitch via the microphone picking up the acoustic room signal (frequency and ring modulation). Lower half of the pot controls volume and filter at `100 Hz`; upper half sweeps pitch up to `800 Hz`.
* **`Self-FM Touch`:** Extended version of *Self-FM*. Using touch pins **`D32`** (next) and **`D4`** (previous), the potentiometer cycles through 4 parameter pages with soft-takeover (pickup) logic: `1: JOURNEY` (level/pitch), `2: SHAPE` (sawtooth to square), `3: FM` depth, and `4: RING` modulation.
* **`Fireflies Pentatonic`:** Coupled oscillators interacting across the room. Each board plays notes from an A-minor pentatonic scale and accelerates its internal clock whenever it hears neighboring boards through the microphone (swarm synchronization). The pot controls pitch, envelope character (pad to percussive), and tempo.
* **`Tape Loop`:** Sound-on-sound tape looper running in the ESP32's RAM. Pot = feedback/erase multiplier; hold **`D4`** = record; hold **`D32`** = pot controls playback speed (`0.5x` to `2.0x`).
* **`FB_Pure_touch`:** Combines *Modal Feedback* with a waveform-aligned granular pitch shifter (WSOLA) and a tape delay directly inside the feedback loop. Hold **`D4`** = pitch (`0.9–1.1`), hold **`D32`** = delay time (`0–1000 ms`), hold **both** = reset pitch to `1.0`.
* **`Piezo / Line Passthrough`:** Low-latency audio passthrough (`50 kHz` sample rate to push switching noise above the audible range) for analog input **`A1`** (`GPIO 36`), featuring a median filter, oversampling, high-pass filter, and spectral noise gate *(requires an external preamp / voltage divider at the input)*.

---

## 4. License

This repository uses dual licensing to properly cover both hardware designs and firmware code:

* **Software & Firmware (`.ino` sketches):** Licensed under the **Apache License 2.0** — see [`LICENSE`](LICENSE). You may freely use, modify, and distribute the code, provided you retain copyright notices and clearly state any modifications made to the files.
* **Hardware (PCB schematics, layout, Gerber files):** Licensed under the **CERN Open Hardware Licence Version 2 – Weakly Reciprocal (`CERN-OHL-W-2.0`)** — see [`LICENSE-HARDWARE`](LICENSE-HARDWARE). You are free to manufacture, modify, and distribute the hardware, provided you give attribution, document any modifications, and share modified hardware design files under the same license.

---

## 5. Notes

* Both the firmware code and the PCB design were developed with AI assistance.

Copyright © 2026 Leon Eckard

*Funded by Musikfonds e.V. by means of the Federal Government Commissioner for Culture and the Media.*
