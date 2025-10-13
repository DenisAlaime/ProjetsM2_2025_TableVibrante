#include <Arduino.h>

// Paramètres
const uint8_t PWM_PIN = 3;
const float VCC = 5.0;
const float VOUT_MAX = 3.3;
const uint8_t PWM_MAX = 255;
const uint8_t PWM_LIMIT = (uint8_t)(PWM_MAX * VOUT_MAX / VCC); // Limite pour 3.3V

unsigned int freq = 100; // Fréquence en Hz (modifiable)
unsigned long lastUpdate = 0;
uint8_t pwmValue = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Entrez une nouvelle fréquence (en Hz) puis appuyez sur Entrée :");

  // PWM très rapide sur la pin 3 (Timer2, prescaler 1 -> ~62,5 kHz)
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
}

void loop() {
  // Lecture série pour changer la fréquence
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    unsigned int newFreq = input.toInt();
    if (newFreq > 0 && newFreq <= 1000) { // Limite à 1kHz pour la stabilité
      freq = newFreq;
      Serial.print("Nouvelle fréquence : ");
      Serial.println(freq);
      
    }
  }

  // Génération du signal dent de scie
  static unsigned long lastStep = 0;
  static uint8_t step = 0;
  unsigned long period = 1000000UL / freq; // période en µs
  unsigned long now = micros();

  // On découpe la période en PWM_LIMIT étapes pour la dent de scie
  unsigned long stepTime = period / PWM_LIMIT;

  if (now - lastStep >= stepTime) {
    lastStep = now;
    analogWrite(PWM_PIN, step);
    step++;
    if (step > PWM_LIMIT) step = 0;
  }
}