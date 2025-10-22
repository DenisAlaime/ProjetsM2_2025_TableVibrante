#include <Arduino.h>
#include <QuickPID.h>
#include "AS5600.h"
#include "CytronMotorDriver.h"
#include "avdweb_AnalogReadFast.h"

// --- Définitions des constantes ---
#define MOTOR1_PIN_A 3
#define MOTOR1_PIN_B 2

#define AS5600_MOT1_DIR_PIN 10
#define SENSOR1_MODE_PIN 9
#define SENSOR1_OUT_POS_PIN A5

#define MOTOR2_PIN_A 4
#define MOTOR2_PIN_B 5

#define AS5600_MOT2_DIR_PIN 11
#define SENSOR2_MODE_PIN 12
#define SENSOR2_OUT_POS_PIN A4

#define REF_INTERRUPT_PIN 6


#define MOTOR_SPEED_MAX 180      // Vitesse max (%)
#define MOTOR_SPEED_MIN 0        // Vitesse min (%)

#define RECORD_N_SAMPLES 1000  // Nombre d'échantillons à enregistrer pour l'analyse
#define RECORD_COLUMNS 4       // Nombre de colonnes à enregistrer (ex : temps, consigne, mesure)

#define RECORD_SMOOTHING 5     // Lissage des enregistrements

#dekfine PID_COMPUTE_PERIOD_US 1000  // Période de calcul des PID en microsecondes

// --- Initialisations des objets ---
CytronMD motor1(PWM_PWM, MOTOR1_PIN_A, MOTOR1_PIN_B);
CytronMD motor2(PWM_PWM, MOTOR2_PIN_A, MOTOR2_PIN_B);

// capteurs AS5600
AS5600 as5600Mot1;
AS5600 as5600Mot2;
float sensor1Scaling = 0.8425;//échelle sur la lecture du capteur. 1.0 signifie 1024 == 360.0°.(pas correct, à calibrer)
float sensor2Scaling = 0.8425;//échelle sur la lecture du capteur. 1.0 signifie 1024 == 360.0°.(pas correct, à calibrer)


// création du buffer pour l'enregistrement des données
float dataBuffer[RECORD_N_SAMPLES][RECORD_COLUMNS];
int recordIndex = 0;
//nom des colonnes
const char* columnNames[RECORD_COLUMNS] = {"Time [ms]", "motor1Output", "Sensor1", "Error1"};

// création des variables pour les capteurs et les PID
float sensor1 = 0.0;
float sensor2 = 0.0;
float motor1Output = 0.0;
float motor2Output = 0.0;

float error1 = 0.0;
float error2 = 0.0;

//les setpoints sont 0 tout le temps, car on utilise l'erreur entre la position mesurée et la position de référence
// cela permet de gérer le wrap-around facilement.
float setPoint1 = 0.0;
float setPoint2 = 0.0; 

// décalage de phase pour le moteur 2
float motor2PhaseOffset = 0.0; 

QuickPID pid1(&error1, &motor1Output, &setPoint1, 0.5, 0.1, 0.05, QuickPID::Action::reverse);
QuickPID pid2(&error2, &motor2Output, &setPoint2, 0.5, 0.1, 0.05, QuickPID::Action::reverse);

// --- Commandes utilisateur ---
float userSpeed = 50.0; // vitesse cible par défaut [Hz]
// période de rotation des moteurs, exprimée en microsecondes
unsigned long period = 20000; // période initiale (50 Hz)
bool motorRunning = true;
// Inversion par défaut pour chaque moteur. motor1 était précédemment inversé dans le code.
bool motor1Inverted = false;
bool motor2Inverted = true;
bool debugMotors = false;
bool changed = true;

unsigned long lastTime = 0;


void handleSerialCommands();
float genReferenceAngle(unsigned long timeMicros);

void setup()
{
    Serial.begin(115200);
    Serial.println();
    
    Wire.begin();
    Wire.setClock(400000UL);

    as5600Mot1.begin(AS5600_MOT1_DIR_PIN);
    as5600Mot1.setDirection(AS5600_COUNTERCLOCK_WISE);
    as5600Mot1.setOutputMode(AS5600_OUTMODE_ANALOG_100);
    pinMode(SENSOR1_MODE_PIN, OUTPUT);
    digitalWrite(SENSOR1_MODE_PIN, HIGH); // mode position

    as5600Mot2.begin(AS5600_MOT2_DIR_PIN);
    as5600Mot2.setDirection(AS5600_COUNTERCLOCK_WISE);
    as5600Mot2.setOutputMode(AS5600_OUTMODE_ANALOG_100);
    pinMode(SENSOR2_MODE_PIN, OUTPUT);
    digitalWrite(SENSOR2_MODE_PIN, HIGH); // mode position

    // init de l'entrée analogique rapide    
    analogReadResolution(10);
    pinMode(SENSOR1_OUT_POS_PIN, INPUT);
    pinMode(SENSOR2_OUT_POS_PIN, INPUT);
    analogRead(SENSOR1_OUT_POS_PIN);
    analogRead(SENSOR2_OUT_POS_PIN);

    // configuration des PID
    pid1.SetOutputLimits(0, MOTOR_SPEED_MAX);
    pid2.SetOutputLimits(0, MOTOR_SPEED_MAX);

    pid1.SetMode(QuickPID::Control::timer);
    pid2.SetMode(QuickPID::Control::timer);

    //library expects fast calls and will sample time itself.
    pid1.SetSampleTimeUs(PID_COMPUTE_PERIOD_US);
    pid2.SetSampleTimeUs(PID_COMPUTE_PERIOD_US);
    delay(1000);
    Serial.println(pid1.Compute());
    Serial.println(pid2.Compute());
    lastTime = micros();
}

void loop()
{
    // temps
    unsigned long currentTime = micros();
    if (currentTime - lastTime < PID_COMPUTE_PERIOD_US) {
        return; // attendre la fin de la période
    }
    lastTime = currentTime;


    // lecture des capteurs
    // sensor1 = as5600Mot1.readAngle();
    // sensor2 = as5600Mot2.readAngle();

    //lecture pin analogique, boucle de lecture et moyenne
    for(int i=0; i<RECORD_SMOOTHING; i++){
        // valeur 1023 -> 360 degrés.
        //we convert the 10 bit value in a float degrees value
        sensor1 += (float)analogReadFast(SENSOR1_OUT_POS_PIN)* 360.0 / 1023.0;
        sensor2 += (float)analogReadFast(SENSOR2_OUT_POS_PIN)* 360.0 / 1023.0;
    }
    sensor1 /= RECORD_SMOOTHING;
    sensor2 /= RECORD_SMOOTHING;
    //ajout des offsets
    sensor1 *= sensor1Scaling;
    sensor2 *= sensor2Scaling;

    

    // gestion des commandes série
    handleSerialCommands();

    // calcul de l'erreur
    float refAngle = genReferenceAngle(currentTime);
    error1 = refAngle - sensor1;
    error2 = refAngle + motor2PhaseOffset - sensor2;


    

    // ramener l'erreur dans la plage -180 à 180°
    while (error1 > 180.0) error1 -= 360.0;
    while (error1 < -180.0) error1 += 360.0;

    while (error2 > 180.0) error2 -= 360.0;
    while (error2 < -180.0) error2 += 360.0;

    

    // calcul des sorties PID
    pid1.Compute();
    pid2.Compute();


    // enregistrement des données
    dataBuffer[recordIndex][0] = (float)currentTime;
    dataBuffer[recordIndex][1] = motor1Output;
    dataBuffer[recordIndex][2] = sensor1;
    dataBuffer[recordIndex][3] = error1;
    recordIndex++;
    if (recordIndex >= RECORD_N_SAMPLES) {
        recordIndex = 0; // reset index when buffer is full
    }


    // vérification du mode PID pour mise à jour des moteurs
    if (pid1.GetMode() > 0){// automatic
        changed = true;
    }

    // commande des moteurs
    if (changed){
        if (motorRunning) {
            if (motor1Inverted) {
                motor1.setSpeed((int)(-motor1Output));
            } else {
                motor1.setSpeed((int)motor1Output);
            }
            if (motor2Inverted) {
                motor2.setSpeed((int)(-motor2Output));
            } else {
                motor2.setSpeed((int)motor2Output);
            }
        } else {
            motor1.setSpeed(0);
            motor2.setSpeed(0);
        }
        changed = false;
    }
    //pause pour stabilité
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
          Serial.println("Moteurs démarrés.");
          changed = true;
        }
        else if (cmd == "st")
        {
          motorRunning = false;
          Serial.println("Moteurs arrêtés.");
          changed = true;
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
          changed = true;
        }
        else if (cmd == "inv2")
        {
          motor2Inverted = !motor2Inverted;
          Serial.print("Inversion moteur 2 : ");
          Serial.println(motor2Inverted ? "ON" : "OFF");
          changed = true;
        }
        else if (cmd == "a")
        {
          // toggle PID auto/manual
            if (pid1.GetMode() == 1)// automatic
            {
                pid1.SetMode(QuickPID::Control::manual);
                pid2.SetMode(QuickPID::Control::manual);
                Serial.println("Mode PID : MANUAL");
            }
            else
            {
                pid1.SetMode(QuickPID::Control::automatic);
                pid2.SetMode(QuickPID::Control::automatic);
                Serial.println("Mode PID : AUTOMATIC");
            }
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
              pid1.SetTunings(v, pid1.GetKi(), pid1.GetKd());
              pid2.SetTunings(v, pid2.GetKi(), pid2.GetKd());
              Serial.print("Kp réglé à ");
              Serial.println(pid1.GetKp(), 4);
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
                pid1.SetTunings(pid1.GetKp(), pid1.GetKi(), v);
                pid2.SetTunings(pid2.GetKp(), pid2.GetKi(), v);
              Serial.print("Kd réglé à ");
              Serial.println(pid1.GetKd(), 6);
            }
            else
            {
              Serial.println("Erreur: valeur Kd invalide");
            }
          }
        }
        else if (cmd.length() > 0 && (cmd.charAt(0) == 'i' || cmd.charAt(0) == 'I'))
        {
          // commande i<val> pour changer le Ki du régulateur
          String num = cmd.substring(1);
          num.trim();
          if (num.length() > 0)
          {
            float v = num.toFloat();
            // Ki peut être 0 ou positif
            if (v >= 0)
            {
                pid1.SetTunings(pid1.GetKp(), v, pid1.GetKd());
                pid2.SetTunings(pid2.GetKp(), v, pid2.GetKd());
              Serial.print("Ki réglé à ");
              Serial.println(pid1.GetKi(), 6);
            }
            else
            {
              Serial.println("Erreur: valeur Ki invalide");
            }
          }
        }
        else if (cmd.length() > 0 && (cmd.charAt(0) == 's' || cmd.charAt(0) == 'S'))
        {
          // commande s<val> pour changer la référence de vitesse du régulateur
          String num = cmd.substring(1);
          num.trim();
          if (num.length() > 0)
          {
            float v = num.toFloat();
            // Ki peut être 0 ou positif
            if (v >= 3.0)
            {
              userSpeed = v;
              period = (unsigned long)(1000000.0 / userSpeed);
              Serial.print("Vitesse de référence [Hz] réglée à ");
              Serial.println(userSpeed, 6);
            }
            else
            {
              Serial.println("Erreur: valeur de vitesse de référence invalide");
            }
          }
        }
        else if (cmd.length() > 0 && (cmd.charAt(0) == 'o' || cmd.charAt(0) == 'O'))
        {
          // commande o<val> pour changer le déphasage du moteur 2 (moteur 1 a toujours 0°)
          String num = cmd.substring(1);
          num.trim();
          if (num.length() > 0)
          {
            float v = num.toFloat();
            // déphasage entre 0 et 360°
            if (v >= 0.0 && v < 360.0)
            {
              motor2PhaseOffset = v;
              Serial.print("Déphasage moteur 2 réglé à ");
              Serial.println(v, 2);
            }
            else
            {
              Serial.println("Erreur: valeur de déphasage invalide");
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
            motor1Output = val>0 ? val : -val;
            if (val < 0) motor1Inverted = !motor1Inverted;
            motor2Output = val>0 ? val : -val;  
            if (val < 0) motor2Inverted = !motor2Inverted;
            Serial.print("Vitesse moteurs réglée à ");
            Serial.print(motor1Output);
            Serial.println("%");
          }
          changed = true;
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


float genReferenceAngle(unsigned long timeMicros){
    // Génère la référence d'angle utilisée pour sychroniser les moteurs en phase. 
    // La référence d'angle est calculée en fonction du temps écoulé.
    unsigned long mod = timeMicros % period;
    float angle = (mod * 360.0 / period);
    return angle;
}