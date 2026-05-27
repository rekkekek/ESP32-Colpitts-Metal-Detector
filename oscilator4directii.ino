#include <Arduino.h>
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

// mapare direcții: 0=sus, 1=dreapta, 2=jos, 3=stanga
#define NUM_SENSORS 4

// pini bobine: D23(sus), D22(dreapta), D21(jos), D15(stanga)
const int oscPins[NUM_SENSORS] = {23, 22, 21, 15};

// pini leduri: D18(sus), D4(dreapta), D5(jos), D2(stanga)
const int ledPins[NUM_SENSORS] = {18, 4, 5, 2};
String directii[NUM_SENSORS] = {"Sus", "Dreapta", "Jos", "Stanga"};

const pcnt_unit_t pcntUnits[NUM_SENSORS] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};

const int buzzerPin = 19;

// PCNT initializare 
#define PCNT_H_LIM_VAL 20000 
volatile uint32_t multPulses[NUM_SENSORS] = {0, 0, 0, 0}; 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

long baselineFreq[NUM_SENSORS] = {0, 0, 0, 0};
int threshold = 30; // Pragul de alertă în Hz

// EMA Filter
float smoothedFreq[NUM_SENSORS] = {0, 0, 0, 0};
const float emaAlpha = 0.9; 

// functia pt interrupturi 
void IRAM_ATTR pcnt_intr_handler(void *arg) {
  portENTER_CRITICAL_ISR(&timerMux);
  uint32_t intr_status = PCNT.int_st.val; 

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (intr_status & BIT(pcntUnits[i])) { 
      multPulses[i]++; 
      PCNT.int_clr.val = BIT(pcntUnits[i]); 
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// masurarea frecventelor simultana pt cele 4 directii
void measureFrequencies(long* results) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    multPulses[i] = 0; 
    pcnt_counter_pause(pcntUnits[i]);
    pcnt_counter_clear(pcntUnits[i]);
  }
  
  long startTime = esp_timer_get_time(); 
  for (int i = 0; i < NUM_SENSORS; i++) {
    pcnt_counter_resume(pcntUnits[i]);
  }
  
  delay(50); 
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    pcnt_counter_pause(pcntUnits[i]);
  }
  long endTime = esp_timer_get_time(); 
  
  float actualTimeSeconds = (endTime - startTime) / 1000000.0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int16_t pulses = 0;
    pcnt_get_counter_value(pcntUnits[i], &pulses);
    long totalPulses = pulses + (multPulses[i] * PCNT_H_LIM_VAL);
    results[i] = (long)(totalPulses / actualTimeSeconds); 
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(buzzerPin, OUTPUT);
  noTone(buzzerPin);
  
  // configurare leduri
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(ledPins[i], OUTPUT);
    analogWrite(ledPins[i], 0); // stingem ledurile
  }

  // configurare PCNT pentru fiecare senzor 
  for (int i = 0; i < NUM_SENSORS; i++) {
    pcnt_config_t pcnt_config = {
      .pulse_gpio_num = oscPins[i],
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC, 
      .neg_mode = PCNT_COUNT_DIS,
      .counter_h_lim = PCNT_H_LIM_VAL,
      .counter_l_lim = -1,
      .unit = pcntUnits[i],
      .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(pcntUnits[i], 10); 
    pcnt_filter_enable(pcntUnits[i]);
    pcnt_event_enable(pcntUnits[i], PCNT_EVT_H_LIM);
    pcnt_intr_enable(pcntUnits[i]);
  }

  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
  
  Serial.println("Calibrare în curs... Nu mișca mâinile sau aparatul!");
  
  // toate cele 4 leduri se aprind in timpul calibrarii pt feedback vizual
  for(int i = 0; i < NUM_SENSORS; i++) analogWrite(ledPins[i], 255);
  delay(2000); 
  
  // calibrarea frecventelor
  long sumaFrecvente[NUM_SENSORS] = {0, 0, 0, 0};
  long tempResults[NUM_SENSORS];

  for (int iter = 0; iter < 20; iter++) {
    measureFrequencies(tempResults);
    for (int i = 0; i < NUM_SENSORS; i++) {
      sumaFrecvente[i] += tempResults[i];
    }
    Serial.print(".");
  }
  Serial.println();
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    baselineFreq[i] = sumaFrecvente[i] / 20;
    smoothedFreq[i] = baselineFreq[i];
    analogWrite(ledPins[i], 0); 
    
    Serial.print("Baza "); Serial.print(directii[i]); 
    Serial.print(" (Osc D"); Serial.print(oscPins[i]); 
    Serial.print(" / Led D"); Serial.print(ledPins[i]); Serial.print("): ");
    Serial.print(baselineFreq[i]); Serial.println(" Hz");
  }
  Serial.println("Calibrare finalizată!");
}

void loop() {
  long rawFreqs[NUM_SENSORS];
  measureFrequencies(rawFreqs);
  
  long maxDiff = 0;
  
  Serial.print("Semnal Metal: ");
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    // 1. aici calculam frecventa filtrata de EMA si diferenta intre baza si ce am calculat
    smoothedFreq[i] = (emaAlpha * rawFreqs[i]) + ((1.0 - emaAlpha) * smoothedFreq[i]);
    long currentFreq = (long)smoothedFreq[i];
    long freqDifference = abs(currentFreq - baselineFreq[i]);
    
    Serial.print("["); Serial.print(directii[i]); Serial.print(": "); Serial.print(freqDifference); Serial.print("] ");

    // 2. cautam semnalul maxim pentru a seta buzzerul dupa acesta
    if (freqDifference > maxDiff) {
      maxDiff = freqDifference;
    }

    // 3. controlam intensitatile ledurilor in functie de diferentele de frecventa pt fiecare senzor
    if (freqDifference > threshold) {
      // mapam diferenta de la 15 (abia vizibil) la 255 (maxim)
      // ajustam valoarea 600 daca oscilatoarele au diferente mai mari sau mai mici cand metalul este aproape
      int brightness = map(freqDifference, threshold, 600, 15, 255);
      
      // ne asiguram ca valoarea nu depaseste limitele permise (0-255)
      brightness = constrain(brightness, 15, 255); 
      
      analogWrite(ledPins[i], brightness); // aprinderea ledului proportional cu apropierea de centrul bobinei
    } else {
      analogWrite(ledPins[i], 0); // oprim ledul daca diferenta e sub treshold
    }
  }
  Serial.println(); 

  // buzzerul suna doar daca cel puțin un senzor a depaseste pragul
  // pitch-ul se alege dupa cel mai puternic semnal (maxDiff)
  if (maxDiff > threshold) {
    int buzzerPitch = map(maxDiff, threshold, 600, 500, 4000);
    buzzerPitch = constrain(buzzerPitch, 500, 4000);
    tone(buzzerPin, buzzerPitch); 
  } else {
    noTone(buzzerPin);  
  }
}