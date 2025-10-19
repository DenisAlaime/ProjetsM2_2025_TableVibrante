#include <Arduino.h>


// --- Définitions des constantes ---

#define REF_INTERRUPT_PIN 6
// Broche de sortie pour générer un créneau à chaque passage à zéro
#define REF_PULSE_PIN 7
// Durée du créneau en microsecondes




// --- Variables moteur maître ---
volatile unsigned long lastRefTime = 0;
volatile unsigned long refPeriod = 1; //REF_PERIOD_DEFAULT;
volatile bool refDetected = false;
// Flags pour gérer la génération du créneau sans appeler digitalWrite() dans l'ISR
// volatile bool pulseRequest = false;
// volatile unsigned long pulseRequestTime = 0;
// volatile bool pulseActive = false;
// volatile unsigned long pulseEndTime = 0;

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

void isrMoteurMaitre();

void testInterruption();

void setup()
{
  Serial.begin(115200);
  
 
  // Interruption sur pin de référence moteur maître
  pinMode(REF_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(REF_INTERRUPT_PIN), isrMoteurMaitre, FALLING);

  // Broche de sortie pour le créneau de référence
  // pinMode(REF_PULSE_PIN, OUTPUT);

  // digitalWrite(REF_PULSE_PIN, LOW);

  //   pinMode(REF_INTERRUPT_PIN, INP);
  // REF_INTERRUPT_PIN

  
}

void loop()
{


//   if (digitalRead(REF_INTERRUPT_PIN) == 0)
//   {
//     digitalWrite(REF_PULSE_PIN, LOW); // Activer le pulse
//   }
//   else

//     digitalWrite(REF_PULSE_PIN, digitalRead(REF_INTERRUPT_PIN)); // Activer le pulse

  
// }

//digitalWrite(REF_PULSE_PIN, digitalRead(REF_INTERRUPT_PIN)); // Activer le pulse
}

// --- Gestion des commandes série ---


// --- Interruption passage à zéro moteur maître ---
void isrMoteurMaitre()
{
  
  testInterruption();// unsigned long now = micros();
  // refPeriod = now - lastRefTime;
  // lastRefTime = now;
   refDetected = true;
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
