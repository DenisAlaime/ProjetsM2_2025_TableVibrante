#include <Arduino.h>
#include "AS5600.h"
#include "CytronMotorDriver.h"

// --- Matériel ---
AS5600 as5600;
CytronMD motor1(PWM_DIR, 4, 5);  // Moteur 1 : PWM = Pin 4, DIR = Pin 5
CytronMD motor2(PWM_DIR, 6, 7);  // Moteur 2 : PWM = Pin 6, DIR = Pin 7

// --- Paramètres régulation ---
const int PHASE_TARGET_DEG = 90;   // Déphasage cible en degrés
const float Kp = 0.8;              // Gain proportionnel (à ajuster)
const int MOTOR_SPEED_MAX = 100;   // Vitesse max (%)

// --- Variables moteur maître ---
volatile unsigned long lastRefTime = 0;
volatile unsigned long refPeriod = 20000; // µs, valeur par défaut pour éviter division par zéro
volatile bool refDetected = false;

// --- Variables moteur esclave ---
unsigned long lastAngleTime = 0;
float lastAngle = 0;

// --- Commandes utilisateur ---
int userSpeed = 50;
bool motorRunning = true;

// --- Prototypes ---
void handleSerialCommands();
void isrMoteurMaitre();
void updateRegulation();
float getPhaseErrorDeg(float refAngle, float slaveAngle);
float readSlaveAngleDeg();
void setMotorsSpeed(int speed); // <-- modifié ici
void testInterruption();

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin();
  Wire.setClock(400000UL);

  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.setOutputMode(AS5600_OUTMODE_ANALOG_100);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  delay(10);

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  setMotorsSpeed(userSpeed); // <-- modifié ici

  // Interruption sur pin 6 pour le signal de référence moteur maître
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6), isrMoteurMaitre, FALLING);

  Serial.println("Tapez 'sr' pour démarrer, 'st' pour arrêter, ou un nombre (0-100) pour régler la vitesse (%)");
  Serial.println("Déphasage cible : 90°");
}

void loop() {
  handleSerialCommands();
  testInterruption();
  // if (motorRunning) {
  //   updateRegulation();
  // }
  delay(10); // Boucle rapide pour la régulation
}

// --- Gestion des commandes série ---
void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "sr") {
      motorRunning = true;
      setMotorsSpeed(userSpeed); // <-- modifié ici
      Serial.println("Moteurs démarrés.");
    } else if (cmd == "st") {
      motorRunning = false;
      setMotorsSpeed(0); // <-- modifié ici
      Serial.println("Moteurs arrêtés.");
    } else {
      int val = cmd.toInt();
      if (val >= 0 && val <= 100) {
        userSpeed = val;
        if (motorRunning) {
          setMotorsSpeed(userSpeed); // <-- modifié ici
        }
        Serial.print("Vitesse moteurs réglée à ");
        Serial.print(userSpeed);
        Serial.println("%");
      }
    }
  }
}

// --- Interruption passage à zéro moteur maître ---
void isrMoteurMaitre() {
  unsigned long now = micros();
  refPeriod = now - lastRefTime;
  lastRefTime = now;
  refDetected = true;
}

// --- Lecture angle moteur esclave (AS5600) ---
float readSlaveAngleDeg() {
  // Angle en degrés (0-360)
  return as5600.readAngle();
}

// --- Calcul erreur de phase (en degrés, -180 à +180) ---
float getPhaseErrorDeg(float refAngle, float slaveAngle) {
  float error = refAngle - slaveAngle;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

// --- Régulation de vitesse et de phase ---
void updateRegulation() {
  static float phaseTarget = PHASE_TARGET_DEG;
  static float lastPhaseError = 0;
  static int speedCmd = 0;

  // On suppose que le moteur maître fait un tour à chaque interruption
  static float refAngle = 0;
  if (refDetected) {
    refDetected = false;
    refAngle = 0; // Réinitialise l'angle de référence à chaque tour
  } else {
    // Avance l'angle de référence en fonction du temps écoulé depuis le dernier passage à zéro
    unsigned long elapsed = micros() - lastRefTime;
    refAngle = (elapsed / (float)refPeriod) * 360.0;
    if (refAngle > 360) refAngle = 360;
  }

  float slaveAngle = readSlaveAngleDeg();
  float phaseError = getPhaseErrorDeg(refAngle + phaseTarget, slaveAngle);

  // Régulation proportionnelle
  speedCmd = userSpeed + int(Kp * phaseError);
  if (speedCmd > MOTOR_SPEED_MAX) speedCmd = MOTOR_SPEED_MAX;
  if (speedCmd < 0) speedCmd = 0;

  setMotorsSpeed(speedCmd);

  // Affichage debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("RefAngle: "); Serial.print(refAngle, 1);
    Serial.print(" | SlaveAngle: "); Serial.print(slaveAngle, 1);
    Serial.print(" | PhaseErr: "); Serial.print(phaseError, 1);
    Serial.print(" | Cmd: "); Serial.println(speedCmd);
    lastPrint = millis();
  }
}

// --- Commande des deux moteurs ---
void setMotorsSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
}

// --- Test interruption ---
void testInterruption() {
  static unsigned long lastPrint = 0;
  if (refDetected) {
    refDetected = false;
    Serial.print("Interruption détectée à ");
    Serial.print(micros());
    Serial.print(" us, période = ");
    Serial.print(refPeriod);
    Serial.println(" us");
    lastPrint = millis();
  }
}

