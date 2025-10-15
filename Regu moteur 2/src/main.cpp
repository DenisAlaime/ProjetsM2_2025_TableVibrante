#include <Arduino.h>
#include "AS5600.h"
#include "CytronMotorDriver.h"

// --- Définitions des constantes ---
#define MOTOR1_PWM_PIN        4
#define MOTOR1_DIR_PIN        5
#define MOTOR2_PWM_PIN        3
#define MOTOR2_DIR_PIN        2
#define AS5600_MOT1_DIR_PIN   11
#define AS5600_MOT2_DIR_PIN   10
#define SENSOR1_MODE_PIN      12
#define SENSOR2_MODE_PIN      9
#define REF_INTERRUPT_PIN     6

#define PHASE_TARGET_DEG      90    // Déphasage cible en degrés
#define KP                    0.8   // Gain proportionnel
#define MOTOR_SPEED_MAX       255   // Vitesse max (%)
#define MOTOR_SPEED_MIN       0     // Vitesse min (%)
#define REF_PERIOD_DEFAULT    20000 // µs, valeur par défaut
#define PRINT_INTERVAL_MS     200   // Affichage debug

// --- Matériel ---
AS5600 as5600Mot1;
AS5600 as5600Mot2;
CytronMD motor1(PWM_DIR, MOTOR1_PWM_PIN, MOTOR1_DIR_PIN);  // Moteur 1
CytronMD motor2(PWM_DIR, MOTOR2_PWM_PIN, MOTOR2_DIR_PIN);  // Moteur 2

// --- Variables moteur maître ---
volatile unsigned long lastRefTime = 0;
volatile unsigned long refPeriod = REF_PERIOD_DEFAULT;
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
void setMotorsSpeed(int speed);
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

  as5600Mot1.begin(AS5600_MOT1_DIR_PIN);
  as5600Mot1.setDirection(AS5600_COUNTERCLOCK_WISE);
  as5600Mot1.setOutputMode(AS5600_OUTMODE_ANALOG_100);

  as5600Mot2.begin(AS5600_MOT2_DIR_PIN);
  as5600Mot2.setDirection(AS5600_COUNTERCLOCK_WISE);
  as5600Mot2.setOutputMode(AS5600_OUTMODE_ANALOG_100);

  pinMode(SENSOR1_MODE_PIN, OUTPUT);
  digitalWrite(SENSOR1_MODE_PIN, HIGH);

  pinMode(SENSOR2_MODE_PIN, OUTPUT);
  digitalWrite(SENSOR2_MODE_PIN, HIGH);

  delay(10);

  setMotorsSpeed(userSpeed);

  // Interruption sur pin de référence moteur maître
  pinMode(REF_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REF_INTERRUPT_PIN), isrMoteurMaitre, FALLING);

  Serial.println("Tapez 'sr' pour démarrer, 'st' pour arrêter, ou un nombre (0-100) pour régler la vitesse (%)");
  Serial.print("Déphasage cible : ");
  Serial.print(PHASE_TARGET_DEG);
  Serial.println("°");
}

void loop() {
  handleSerialCommands();
  testInterruption();
  if (motorRunning) {
     updateRegulation();
   }
  //delay(10);
}

// --- Gestion des commandes série ---
void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "sr") {
      motorRunning = true;
      setMotorsSpeed(userSpeed);
      Serial.println("Moteurs démarrés.");
    } else if (cmd == "st") {
      motorRunning = false;
      setMotorsSpeed(0);
      Serial.println("Moteurs arrêtés.");
    } else {
      int val = cmd.toInt();
      if (val >= MOTOR_SPEED_MIN && val <= MOTOR_SPEED_MAX) {
        userSpeed = val;
        if (motorRunning) {
          setMotorsSpeed(userSpeed);
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
  return as5600Mot1.readAngle();
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
  speedCmd = userSpeed + int(KP * phaseError);
  if (speedCmd > MOTOR_SPEED_MAX) speedCmd = MOTOR_SPEED_MAX;
  if (speedCmd < MOTOR_SPEED_MIN) speedCmd = MOTOR_SPEED_MIN;

  setMotorsSpeed(speedCmd);

  // Affichage debug
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > PRINT_INTERVAL_MS) {
  //   Serial.print("RefAngle: "); Serial.print(refAngle, 1);
  //   Serial.print(" | SlaveAngle: "); Serial.print(slaveAngle, 1);
  //   Serial.print(" | PhaseErr: "); Serial.print(phaseError, 1);
  //   Serial.print(" | Cmd: "); Serial.println(speedCmd);
  //   lastPrint = millis();
  // }
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
    // Serial.print("Interruption détectée à ");
    // Serial.print(micros());
    // Serial.print(" us, période = ");
    // Serial.print(refPeriod);
    // Serial.println(" us");
    // lastPrint = millis();
  }
}

