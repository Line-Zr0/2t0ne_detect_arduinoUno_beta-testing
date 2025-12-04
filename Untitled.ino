#include <Wire.h>  
#include <LCD-I2C.h>

LCD_I2C lcd(0x27, 16, 2); 




/*
  Two-Tone Pager Receiver (Simplified, Latched ALERT)
  ---------------------------------------------------
  - Audio from Baofeng -> analog front end -> A0
  - Detects TONE1 followed by TONE2 using Goertzel.
  - No LED, no buzzer, no low-power mode.
  - Prints state and debug values to Serial (9600 baud).
  - When ALERT is entered, it stays latched until you manually reset:
      - Send 'r' or 'R' over Serial, OR
      - Press the Arduino hardware reset button.
*/

#include <Arduino.h>

// ----------------- USER CONFIG -----------------

// Target tone frequencies (Hz)
const float TONE1_FREQ = 600.0;
const float TONE2_FREQ = 800.0;

// Sampling
const float SAMPLE_RATE = 8000.0;      // Nominal 10 kHz
const int   N_SAMPLES  = 500;          // Samples per block (~20 ms at 10 kHz)


// Thresholds
const float TONE_LEVEL_THRESHOLD = 0.003;     // Adjust after calibration
const int   NOISE_FLOOR          = 5;        // Average absolute ADC counts below this = "silence"

// ----------------- PIN ASSIGNMENTS -----------------

const int AUDIO_PIN  = A2;
const int BUZZ_PIN = 2;
const int BUZZ_PIN1 = 3;


// ----------------- INTERNALS -----------------

// Goertzel coefficients
float coeff1, coeff2;

// Sample buffer
int16_t samples[N_SAMPLES];

// State machine
enum PagerState {
  IDLE = 0,
  WAITING_TONE2,
  ALERT
};

PagerState currentState = IDLE;
void updateLCDState();

// Debug timing
unsigned long lastDebugPrintTime = 0;

// ----------------- GOERTZEL SETUP -----------------

float computeGoertzelCoeff(float targetFreq) {
  // For block size N_SAMPLES, sampling SAMPLE_RATE
float k = round((N_SAMPLES * targetFreq) / SAMPLE_RATE);
  float omega = (2.0 * PI * k) / N_SAMPLES;
  float coeff = 2.0 * cos(omega);
  return coeff;
}

float goertzelMagnitudeSquared(float coeff) {
  // Run Goertzel on global 'samples' buffer
  float s_prev = 0;
  float s_prev2 = 0;

  for (int i = 0; i < N_SAMPLES; i++) {
    float s = (float)samples[i] + coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }

  float magnitude2 = s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2;
  return magnitude2;
}

// Convert raw magnitude^2 into a rough 0..1 "level"
float magnitudeToLevel(float mag2) {
  float mag = sqrt(mag2);
 float level = mag / 5000.0; // 512 ≈ typical max ADC swing
  if (level > 1.0) level = 1.0;
  return level;
}

// ----------------- HELPERS -----------------

void resetPagerState() {
  currentState = IDLE;
   updateLCDState();
  Serial.println(F("MANUAL RESET -> State set to IDLE"));
}

void checkSerialReset() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      resetPagerState();
    }
  }
  
}


void updateLCDState() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Two-Tone Pager");

  lcd.setCursor(0, 1);
  lcd.print("State: ");

  switch (currentState) {
    case IDLE:
      lcd.print("IDLE");
      break;
    case WAITING_TONE2:
      lcd.print("WAIT T2");
      break;
    case ALERT:
      lcd.print("ALERT");
      break;
  }
}


// ----------------- SETUP -----------------

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(A0, INPUT);

  Serial.println(F("Two-Tone Pager Receiver (Simplified, Latched ALERT)"));
  Serial.println(F("Config:"));
  Serial.print(F("  TONE1_FREQ = ")); Serial.println(TONE1_FREQ);
  Serial.print(F("  TONE2_FREQ = ")); Serial.println(TONE2_FREQ);
  Serial.print(F("  SAMPLE_RATE = ")); Serial.println(SAMPLE_RATE);
  Serial.print(F("  N_SAMPLES   = ")); Serial.println(N_SAMPLES);

  Serial.println(F("Send 'r' over Serial to reset back to IDLE when in ALERT."));

  // -------- LCD INIT --------
  Wire.begin();        
  lcd.begin(&Wire);    
  lcd.display();    
  lcd.backlight();  
  updateLCDState(); 
  // --------------------------

  // Precompute Goertzel coefficients
  coeff1 = computeGoertzelCoeff(TONE1_FREQ);
  coeff2 = computeGoertzelCoeff(TONE2_FREQ);

  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(BUZZ_PIN1, OUTPUT);
}

// ----------------- MAIN LOOP -----------------

void loop() {
   if(digitalRead(4)== LOW){
         resetPagerState();}

  checkSerialReset(); // Allow manual reset at any time

  unsigned long now = millis();

  // Acquire one block of samples from A0 at ~10kHz
  unsigned long usPerSample = (unsigned long)(1000000.0 / SAMPLE_RATE);
  long sumAbs = 0;

  for (int i = 0; i < N_SAMPLES; i++) {
    unsigned long tSampleStart = micros();

    int raw = analogRead(A0);   // 0..1023
    int centered = raw - 740;         // approx center
    samples[i] = (int16_t)centered;
    sumAbs += abs(centered);

    // Hold timing to maintain roughly constant sample rate
    while ((micros() - tSampleStart) < usPerSample) {
      // busy wait
    }
  }

  // Approx block duration in ms
  unsigned long blockDurationMs = (N_SAMPLES * 1000UL) / (unsigned long)SAMPLE_RATE;

  // Determine if block has "real signal" above noise floor
  float avgAbs = (float)sumAbs / (float)N_SAMPLES;
  bool hasSignal = (avgAbs > NOISE_FLOOR);

  float level1 = 0.0;
  float level2 = 0.0;

  if (hasSignal) {
    // Run Goertzel for both tones
    float mag2_1 = goertzelMagnitudeSquared(coeff1);
    float mag2_2 = goertzelMagnitudeSquared(coeff2);

    level1 = magnitudeToLevel(mag2_1);
    level2 = magnitudeToLevel(mag2_2);
  }

  // ---- STATE MACHINE ----
  switch (currentState) {
    case IDLE: {
      digitalWrite(BUZZ_PIN,LOW);
      digitalWrite(BUZZ_PIN1,LOW);
      // If tone1 strongly present, accumulate time
      if (level1 > TONE_LEVEL_THRESHOLD) {
    

        currentState = WAITING_TONE2;
      
        Serial.println(F("TRANSITION: IDLE -> WAITING_TONE2 (Tone1 detected)"));
        updateLCDState();
      }
      break;
    }

    case WAITING_TONE2: {
    
     
      if (level2 > TONE_LEVEL_THRESHOLD) {
       currentState = ALERT;

       Serial.println(F("TRANSITION: WAITING_TONE2 -> ALERT (Tone2 detected)"));
       updateLCDState();
      }
      break;
      }



    case ALERT: {
      // Latched ALERT: do nothing here.
      // We stay in ALERT until manual reset via:
      //   - 'r' / 'R' over Serial (see checkSerialReset), or
      //   - Arduino hardware reset button.
      delay(10);
      if(digitalRead(4)== LOW){
         resetPagerState();}
      digitalWrite(BUZZ_PIN,HIGH);
      digitalWrite(BUZZ_PIN1,HIGH);
      delay(1000);
      if(digitalRead(4)== LOW){
         resetPagerState();}
      digitalWrite(BUZZ_PIN,LOW);
      
          

      
      
      break;
    }
  }

  // ---- DEBUG PRINTING ----
  if ((now - lastDebugPrintTime) > 200) { // every ~200 ms
    Serial.print(F("STATE="));
    switch (currentState) {
      case IDLE:           Serial.print(F("IDLE")); break;
      case WAITING_TONE2:  Serial.print(F("WAITING_TONE2")); break;
      case ALERT:          Serial.print(F("ALERT")); break;
    }
    Serial.print(F("  avgAbs="));
    Serial.print(avgAbs, 1);
    Serial.print(F("  L1="));
    Serial.print(level1, 3);
    Serial.print(F("  L2="));
    Serial.print(level2, 3);
    
    // --- RAW DEBUG PRINT ---
     int raw = analogRead(AUDIO_PIN);  // 0..1023
    Serial.print("___RAW=");
    Serial.print(raw);
    Serial.print("  ");
    Serial.println();

    lastDebugPrintTime = now;
  }
}

