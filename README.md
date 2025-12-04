# 2t0ne_detect_arduinoUno_beta-testing
twoToneDetectForArduinoUno¡usingGeoertzalAndDFTToDetect2SpecifiedHertzTonesInSequenceForRadioPagerMonitering
Two-Tone Pager Receiver (Simplified, Latched Alert)

This Arduino-based project detects two sequential paging tones (commonly used in fire/EMS pagers) using the Goertzel algorithm, then latches into an ALERT state until manually reset. It also provides LCD feedback, serial debugging, and output pins for buzzing/relays/etc.

This project is designed for experimentation with tone-based alerting systems and improving the reliability of analog radio tone decoding.

Features

Detects Tone 1 → Tone 2 in sequence

Uses a customizable Goertzel DSP detector

LCD display shows current state

Audible/relay alert output (latched)

Manual reset via serial, hardware button, or reset pin

Full debug output over serial

Adjustable thresholds and sample settings

Code Overview (Detailed Breakdown)

Below is a deep dive into each subsystem and the logic behind it.

1. Included Libraries
#include <Wire.h>
#include <LCD-I2C.h>
#include <Arduino.h>

What they do:

Wire.h → Controls I²C communication (needed for LCD).

LCD-I2C.h → Talks to a 16x2 LCD display over I²C.

Arduino.h → Standard Arduino definitions, macros, and types used throughout the program.

2. User Configuration Block
const float TONE1_FREQ = 600.0;
const float TONE2_FREQ = 800.0;

const float SAMPLE_RATE = 8000.0;
const int   N_SAMPLES  = 500;

const float TONE_LEVEL_THRESHOLD = 0.003;
const int   NOISE_FLOOR          = 5;

Purpose:

These variables define the behavior of the tone detector.

TONE1_FREQ / TONE2_FREQ: The two pager tones you want to detect.

SAMPLE_RATE: Approximate frequency at which samples are taken.

N_SAMPLES: Length of each sample block processed by Goertzel.
Determines detection time window (~20 ms blocks).

TONE_LEVEL_THRESHOLD: Minimum strength required to consider a tone "present".

NOISE_FLOOR: Minimum average amplitude to consider the signal “real” and not silence.

All of these should be tuned to your radio + audio front-end.

3. Pin Assignments
const int AUDIO_PIN  = A2;
const int BUZZ_PIN   = 2;
const int BUZZ_PIN1  = 3;

Explanation:

AUDIO_PIN (A2) → Raw audio from Baofeng/receiver after analog filtering.

BUZZ_PIN / BUZZ_PIN1 → Drive buzzers, relays, LEDs, or other alert hardware.

4. Internal Storage and Processing Variables
float coeff1, coeff2;
int16_t samples[N_SAMPLES];


coeff1 / coeff2 → Precomputed Goertzel coefficients for each target tone
(avoid recomputing every loop).

samples[] → Buffer storing raw 16-bit samples for DSP analysis.

5. State Machine Definition
enum PagerState {
  IDLE,
  WAITING_TONE2,
  ALERT
};
PagerState currentState = IDLE;

States:

IDLE → Waiting for Tone 1

WAITING_TONE2 → Tone 1 detected; now waiting for Tone 2

ALERT → Both tones detected; alert latched until reset

This keeps detection clean and deterministic.

6. Goertzel Algorithm Functions
6.1 Compute Coefficient
float computeGoertzelCoeff(float targetFreq)


Calculates the DSP coefficient used to detect a specific frequency. This is done once in setup() for performance.

6.2 Run Goertzel and Get Magnitude
float goertzelMagnitudeSquared(float coeff)


Processes the entire samples[] buffer

Implements the Goertzel recurrence formula

Returns frequency magnitude squared (increased = tone present)

6.3 Convert Magnitude to Normalized Level
float magnitudeToLevel(float mag2)


Converts Goertzel output into a 0–1 range.
Used for threshold comparisons.

7. Helper Functions
7.1 Reset Function
void resetPagerState()


Resets state machine to IDLE and updates LCD.

7.2 Check Serial for Reset
void checkSerialReset()


Allows manual reset without physical interaction.

7.3 LCD Display Updater
void updateLCDState()


Displays the current state (IDLE / WAIT T2 / ALERT) on a 16×2 I²C LCD.

8. Setup Function
void setup()

Major actions:

Initialize Serial

Print configuration values

Initialize LCD (I²C)

Precompute Goertzel coefficients

Configure pin modes

After this, the system is ready to start sampling.

9. Main Loop (Core Logic)

This is where real-time tone detection happens.

9.1 Manual Reset Check
if (digitalRead(4) == LOW) resetPagerState();
checkSerialReset();


Allows hardware or serial reset at any time.

9.2 Sampling Audio

The loop captures 500 samples at ~8000 Hz:

unsigned long usPerSample = (1000000.0 / SAMPLE_RATE);
int raw = analogRead(A0);
int centered = raw - 740;
samples[i] = centered;

Notes:

Audio is "centered" by removing the DC offset (~740).

A busy-wait loop ensures consistent sample timing.

9.3 Determine Noise vs Real Audio
float avgAbs = (float)sumAbs / N_SAMPLES;
bool hasSignal = (avgAbs > NOISE_FLOOR);


If the signal amplitude is below the noise floor, Goertzel is skipped to save time.

9.4 Run Tone Detection
float mag2_1 = goertzelMagnitudeSquared(coeff1);
float mag2_2 = goertzelMagnitudeSquared(coeff2);


Converted to normalized levels:

level1 = magnitudeToLevel(mag2_1);
level2 = magnitudeToLevel(mag2_2);

10. State Machine Logic
IDLE → WAITING_TONE2

Triggered when Tone 1 is detected above threshold.

WAITING_TONE2 → ALERT

Triggered when Tone 2 is detected after Tone 1.

ALERT (Latched)
digitalWrite(BUZZ_PIN, HIGH);
digitalWrite(BUZZ_PIN1, HIGH);
delay(1000);


Stays active until reset by:

Serial 'r'

Digital pin 4 input

Hardware reset button

11. Debug Output

Every 200 ms the system prints its status:

STATE=WAITING_TONE2  avgAbs=512.0  L1=0.026  L2=0.013  RAW=...


This provides:

Current state

Average signal amplitude

Tone detection levels

Raw ADC reading

Useful for calibration and troubleshooting.
