#include <Arduino.h>
#include "AS5600.h"
#include "CytronMotorDriver.h"

// --- Définitions des constantes ---
#define MOTOR1_PWM_PIN 3
#define MOTOR1_DIR_PIN 2
#define MOTOR2_PWM_PIN 4
#define MOTOR2_DIR_PIN 5
#define AS5600_MOT1_DIR_PIN 11
#define AS5600_MOT2_DIR_PIN 10
#define SENSOR1_MODE_PIN 12
#define SENSOR2_MODE_PIN 9
#define REF_INTERRUPT_PIN 6
// Broche de sortie pour générer un créneau à chaque passage à zéro
#define REF_PULSE_PIN 7
// Durée du créneau en microsecondes
#define REF_PULSE_WIDTH_US 25

#define PHASE_TARGET_DEG 0 // Déphasage cible en degrés
// Gains du régulateur (modifiable à l'exécution)
float Kp = 0.8;                  // proportionnel
float Kd = 0.0;                  // dérivé
#define MOTOR_SPEED_MAX 255      // Vitesse max (%)
#define MOTOR_SPEED_MIN 0        // Vitesse min (%)
#define REF_PERIOD_DEFAULT 20000 // µs, valeur par défaut
#define PRINT_INTERVAL_MS 200    // Affichage debug

// --- Matériel ---
AS5600 as5600Mot1;
AS5600 as5600Mot2;
// création du buffer pour l'enregistrement des données
float dataBuffer[RECORD_N_SAMPLES][RECORD_COLUMNS];
int recordIndex = 0;
//nom des colonnes
const char* columnNames[RECORD_COLUMNS] = {"Time [ms]", "motor1Output", "Sensor1", "Error1"};

// --- Variables moteur esclave ---
unsigned long lastAngleTime = 0;
float lastAngle = 0;

// --- Commandes utilisateur ---
int userSpeed = 50;
bool motorRunning = true;
// Inversion par défaut pour chaque moteur. motor1 était précédemment inversé dans le code.
bool motor1Inverted = true;
bool motor2Inverted = false;
bool debugMotors = false;
// Affiche le KPI (Kp) initial au démarrage si besoin

// --- Prototypes ---
void handleSerialCommands();
void isrMoteurMaitre();
void updateRegulation();
float getPhaseErrorDeg(float refAngle, float slaveAngle);
float readSlaveAngleDeg();
void setMotorsSpeed(int speed);
void testInterruption();

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin();
  Wire.setClock(400000UL);

  as5600Mot1.begin(AS5600_MOT1_DIR_PIN);
  as5600Mot1.setDirection(AS5600_CLOCK_WISE);
  as5600Mot1.setOutputMode(AS5600_OUTMODE_ANALOG_100);

  as5600Mot2.begin(AS5600_MOT2_DIR_PIN);
  as5600Mot2.setDirection(AS5600_COUNTERCLOCK_WISE);
  as5600Mot2.setOutputMode(AS5600_OUTMODE_ANALOG_100);

  pinMode(SENSOR1_MODE_PIN, OUTPUT);
  digitalWrite(SENSOR1_MODE_PIN, HIGH);

  pinMode(SENSOR2_MODE_PIN, OUTPUT);
  digitalWrite(SENSOR2_MODE_PIN, HIGH);

  delay(10);

  // setMotorsSpeed(userSpeed);

  // Interruption sur pin de référence moteur maître
  pinMode(REF_INTERRUPT_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(REF_INTERRUPT_PIN), isrMoteurMaitre, FALLING);

  // Broche de sortie pour le créneau de référence
  pinMode(REF_PULSE_PIN, OUTPUT);

  digitalWrite(REF_PULSE_PIN, LOW);

  //   pinMode(REF_INTERRUPT_PIN, INP);
  // REF_INTERRUPT_PIN

  Serial.println("Tapez 'sr' pour démarrer, 'st' pour arrêter, ou un nombre (0-100) pour régler la vitesse (%)");
  Serial.print("Déphasage cible : ");
  Serial.print(PHASE_TARGET_DEG);
  Serial.println("°");
}

void loop()
{
  handleSerialCommands();

    // enregistrement des données
    dataBuffer[recordIndex][0] = (float)currentTime;
    dataBuffer[recordIndex][1] = motor1Output;
    dataBuffer[recordIndex][2] = sensor1;
    dataBuffer[recordIndex][3] = error1;
    recordIndex++;
    if (recordIndex >= RECORD_N_SAMPLES) {
        recordIndex = 0; // reset index when buffer is full
    }
  }
  // delay(10);
}

// --- Gestion des commandes série ---
void handleSerialCommands()
{
  // Non bloquant : accumule les caractères dans un buffer et ne traite la commande
  // que lorsque l'utilisateur envoie un CR (\r) ou LF (\n).
  static String inBuf = "";
  const size_t MAX_CMD_LEN = 64;

  while (Serial.available())
  {
    char c = (char)Serial.read();

    // Fin de ligne : traiter la commande si le buffer n'est pas vide
    if (c == '\r' || c == '\n')
    {
      if (inBuf.length() > 0)
      {
        String cmd = inBuf;
        inBuf = "";
        cmd.trim();

        if (cmd == "sr")
        {
          motorRunning = true;
          setMotorsSpeed(userSpeed);
          Serial.println("Moteurs démarrés.");
        }
        else if (cmd == "dbg")
        {
          debugMotors = !debugMotors;
          Serial.print("Debug moteurs : ");
          Serial.println(debugMotors ? "ON" : "OFF");
        }
        else if (cmd == "inv1")
        {
          motor1Inverted = !motor1Inverted;
          Serial.print("Inversion moteur 1 : ");
          Serial.println(motor1Inverted ? "ON" : "OFF");
        }
        else if (cmd == "inv2")
        {
          motor2Inverted = !motor2Inverted;
          Serial.print("Inversion moteur 2 : ");
          Serial.println(motor2Inverted ? "ON" : "OFF");
        }
        else if (cmd.length() > 0 && (cmd.charAt(0) == 'k' || cmd.charAt(0) == 'K'))
        {
          // commande k<val> pour changer le Kp du régulateur
          String num = cmd.substring(1);
          num.trim();
          if (num.length() > 0)
          {
            float v = num.toFloat();
            if (v > 0)
            {
              Kp = v;
              Serial.print("Kp réglé à ");
              Serial.println(Kp, 4);
            }
            else
            {
              Serial.println("Erreur: valeur Kp invalide");
            }
          }
        }
        else if (cmd.length() > 0 && (cmd.charAt(0) == 'd' || cmd.charAt(0) == 'D'))
        {
          // commande d<val> pour changer le Kd du régulateur
          String num = cmd.substring(1);
          num.trim();
          if (num.length() > 0)
          {
            float v = num.toFloat();
            // Kd peut être 0 ou positif
            if (v >= 0)
            {
              Kd = v;
              Serial.print("Kd réglé à ");
              Serial.println(Kd, 6);
            }
            else
            {
              Serial.println("Erreur: valeur Kd invalide");
            }
          }
        }
        else if (cmd == "gr")
        {//plot the buffered content : One line with "GRAPH", then header line, then data lines, then "END"
          Serial.println("GRAPH");
          // print header
          for (int col = 0; col < RECORD_COLUMNS; col++) {
              Serial.print(columnNames[col]);
              if (col < RECORD_COLUMNS - 1) Serial.print(",");
          }
          Serial.println();
          // print data
          int i = recordIndex;
          while (i+1 != recordIndex)
          {
            for (int col = 0; col < RECORD_COLUMNS; col++) {
                Serial.print(dataBuffer[i][col]);
                if (col < RECORD_COLUMNS - 1) Serial.print(",");
            }
            Serial.println();
            i++;
            if (i >= RECORD_N_SAMPLES) i = 0;
        }
          Serial.println("END");
        }
        else
        {
          int val = cmd.toInt();
          // Autoriser une vitesse négative pour inverser le sens de rotation
          if (val >= -MOTOR_SPEED_MAX && val <= MOTOR_SPEED_MAX)
          {
            userSpeed = val;
            if (motorRunning)
            {
              setMotorsSpeed(userSpeed);
            }
            Serial.print("Vitesse moteurs réglée à ");
            Serial.print(userSpeed);
            Serial.println("%");
          }
        }
      }
      else
      {
        // Si on reçoit juste un CR/LF sans contenu, on l'ignore
        inBuf = "";
      }
      // continuer la boucle pour consommer d'éventuels caractères supplémentaires
    }
    // Backspace handling
    else if (c == '\b' || c == 127)
    {
      if (inBuf.length() > 0)
        inBuf.remove(inBuf.length() - 1);
    }
    // Caractère normal : ajouter au buffer si on n'a pas dépassé la taille max
    else
    {
      if (inBuf.length() < MAX_CMD_LEN)
      {
        inBuf += c;
      }
      else
      {
        // Si overflow, on réinitialise le buffer pour éviter comportement indéterminé
        inBuf = "";
        Serial.println("Erreur: commande trop longue, buffer réinitialisé.");
      }
    }
  }
}

// --- Interruption passage à zéro moteur maître ---
void isrMoteurMaitre()
{
  unsigned long now = micros();
  refPeriod = now - lastRefTime;
  lastRefTime = now;
  refDetected = true;
}

// --- Lecture angle moteur esclave (AS5600) ---
float readSlaveAngleDeg()
{
  // Angle en degrés (0-360)
  return as5600Mot2.readAngle();
}

// --- Calcul erreur de phase (en degrés, -180 à +180) ---
float getPhaseErrorDeg(float refAngle, float slaveAngle)
{
  float error = refAngle - slaveAngle;
  while (error > 180)
    error -= 360;
  while (error < -180)
    error += 360;
  return error;
}

// --- Régulation de vitesse et de phase ---
void updateRegulation()
{
  static float phaseTarget = PHASE_TARGET_DEG;
  static float lastPhaseError = 0;
  static int speedCmd = 0;

  // On suppose que le moteur maître fait un tour à chaque interruption
  static float refAngle = 0;
  if (refDetected)
  {
    refDetected = false;
    refAngle = 0; // Réinitialise l'angle de référence à chaque tour
    // digitalWrite(REF_PULSE_PIN, !digitalRead(REF_PULSE_PIN)); // Activer le pulse
  }
  else
  {
    // Avance l'angle de référence en fonction du temps écoulé depuis le dernier passage à zéro
    unsigned long elapsed = micros() - lastRefTime;
    refAngle = (elapsed / (float)refPeriod) * 360.0;
    if (refAngle > 360)
      refAngle = 360;
  }

  float slaveAngle = readSlaveAngleDeg();
  float phaseError = getPhaseErrorDeg(refAngle + phaseTarget, slaveAngle);

  // Régulation PD (proportionnel + dérivé)
  float pTerm = Kp * phaseError;
  float dTerm = Kd * (phaseError - lastPhaseError);
  speedCmd = userSpeed + int(pTerm + dTerm);
  if (speedCmd > MOTOR_SPEED_MAX)
    speedCmd = MOTOR_SPEED_MAX;
  if (speedCmd < MOTOR_SPEED_MIN)
    speedCmd = MOTOR_SPEED_MIN;

  // mettre à jour lastPhaseError pour le terme dérivé
  lastPhaseError = phaseError;

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
void setMotorsSpeed(int speed)
{
  int s1 = speed < 0 ? -speed : speed;
  int s2 = motor2Inverted ? -speed : speed;
  motor1.setSpeed(s1);
  motor2.setSpeed(s2);

  if (debugMotors)
  {
    Serial.print("setMotorsSpeed: consigne=");
    Serial.print(speed);
    Serial.print(" | motor1Inverted=");
    Serial.print(motor1Inverted);
    Serial.print(" -> s1=");
    Serial.print(s1);
    Serial.print(" | motor2Inverted=");
    Serial.print(motor2Inverted);
    Serial.print(" -> s2=");
    Serial.println(s2);
  }
}

// --- Test interruption ---
void testInterruption()
{
  static unsigned long lastPrint = 0;
  if (refDetected == true)
  {
    digitalWrite(REF_PULSE_PIN, !digitalRead(REF_PULSE_PIN)); // Activer le pulse
    refDetected = false;
  }
}
