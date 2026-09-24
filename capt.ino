/*
 * arduino_capteurs.ino  v4 - ultrason + gaz + temperature + vapeur + lumiere
 *
 * Trame serie 115200 bauds, une ligne toutes les ~100 ms, cle=valeur :
 *   c=87 g=-1 d=-1 gaz=312 tadc=512 temp=23.4 vapeur=18 lum=640
 * Un module non branche est DETECTE (rappel interne) et n'ecrit pas sa cle.
 * Cas limite : un module dont S est sur le + (lecture ~5 V) est vu "absent".
 * Le serveur sur le robot (capteurs_serveur.py) publie tout : rien a changer.
 *
 * ============================ FICHE DE CABLAGE =============================
 *   Alimentation : rail 5V et rail GND sur une plaque d'essai, depuis les
 *   broches 5V et GND de l'Uno. TOUS les modules s'alimentent sur ces rails.
 *
 *   MODULE           BROCHE MODULE   ->  ARDUINO
 *   HC-SR04 centre   VCC / GND           rails       TRIG D9    ECHO D10
 *   HC-SR04 gauche   (optionnel)         rails       TRIG D7    ECHO D8
 *   HC-SR04 droite   (optionnel)         rails       TRIG D11   ECHO D12
 *   MQ-x gaz         VCC / GND           rails       AO -> A0   (DO : rien)
 *   Temperature NTC  + / -               rails       S  -> A1
 *   Temperature DHT11 + / -              rails       DATA -> D4 (TEMP_TYPE 1)
 *   Steam sensor     + / -               rails       S  -> A2
 *   Lumiere LDR      + / -               rails       S  -> A3
 *
 *   ATTENTION modules 3 broches : l'ordre S/+/- CHANGE selon le fabricant.
 *   Lire la serigraphie. Un S branche a la place du - donne ~1020 (ou ~0).
 * ===========================================================================
 *
 * TEMPERATURE : TEMP_TYPE 0 = module analogique a thermistance NTC 10k (3 broches)
 *               TEMP_TYPE 1 = DHT11 numerique (installer "DHT sensor library"
 *                             et "Adafruit Unified Sensor" via le gestionnaire)
 *   tadc = ADC brut de A1 (diagnostic, TEMP_TYPE 0) : ~300-700 correct,
 *          ~1020 ou ~0 = cablage. temp n'est publie que s'il est plausible.
 *   Si temp BAISSE quand vous chauffez le capteur : INVERSER_TEMP = true.
 */

#define TEMP_TYPE 0

const int  NB_ULTRASONS   = 1;       // 1, 2 ou 3
// Detection AUTOMATIQUE des modules analogiques : une broche en l'air monte a
// ~5 V quand on active sa resistance de rappel interne, un module branche
// impose sa tension et ne bouge pas. Un module absent n'ecrit pas sa cle.
// Mettre false pour ignorer un module meme branche.
const bool GAZ_ACTIF    = true;
const bool TEMP_ACTIF   = true;
const bool VAPEUR_ACTIF = true;
const bool LUM_ACTIF    = true;
const int  SEUIL_ABSENT = 1005;      // ADC >= ceci avec pull-up = rien de branche

const int TRIG[3] = {9, 7, 11};      // centre, gauche, droite
const int ECHO[3] = {10, 8, 12};
const int BROCHE_GAZ    = A0;
const int BROCHE_TEMP   = A1;
const int BROCHE_VAPEUR = A2;
const int BROCHE_LUM    = A3;
const int BROCHE_DHT    = 4;

// thermistance NTC (TEMP_TYPE 0)
const float R_SERIE = 10000.0;
const float R_25    = 10000.0;
const float BETA    = 3950.0;
const bool  INVERSER_TEMP = false;

const unsigned long TIMEOUT_US = 25000;   // 25 ms ~ 4,3 m aller-retour
const int PORTEE_MAX_CM = 400;

#if TEMP_TYPE == 1
  #include <DHT.h>
  DHT dht(BROCHE_DHT, DHT11);
  unsigned long derniereLectureDht = 0;
  float derniereTempDht = NAN;
#endif

// ---------------------------------------------------------------------------

long mesurerUltrason(int i) {
  digitalWrite(TRIG[i], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG[i], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);
  unsigned long duree = pulseIn(ECHO[i], HIGH, TIMEOUT_US);
  if (duree == 0) return -1;                 // pas d'echo : rien a moins de 4 m
  long cm = duree / 58;
  return (cm > PORTEE_MAX_CM) ? -1 : cm;
}

bool moduleBranche(int broche) {
  pinMode(broche, INPUT_PULLUP);             // rappel interne ~35 kOhm vers 5 V
  delayMicroseconds(300);
  analogRead(broche);
  int avecRappel = analogRead(broche);
  pinMode(broche, INPUT);                    // on relache aussitot
  delayMicroseconds(300);
  return avecRappel < SEUIL_ABSENT;          // un module reel tient sa tension
}

int moyenneAnalogique(int broche) {
  analogRead(broche);                        // premiere lecture jetee (multiplexeur)
  long somme = 0;
  for (int i = 0; i < 8; i++) somme += analogRead(broche);
  return somme / 8;
}

int dernierAdcTemp = -1;

float lireTemperature() {
#if TEMP_TYPE == 1
  if (millis() - derniereLectureDht > 2000) {   // le DHT11 ne repond que toutes les 2 s
    derniereLectureDht = millis();
    derniereTempDht = dht.readTemperature();
  }
  return derniereTempDht;
#else
  int adc = moyenneAnalogique(BROCHE_TEMP);
  dernierAdcTemp = adc;
  if (adc <= 2 || adc >= 1000) return NAN;    // absent, HS, ou ligne numerique
  float r = INVERSER_TEMP ? R_SERIE * adc / (1023.0 - adc)
                          : R_SERIE * (1023.0 / adc - 1.0);
  float invT = 1.0 / 298.15 + log(r / R_25) / BETA;
  return 1.0 / invT - 273.15;
#endif
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT); pinMode(ECHO[i], INPUT); digitalWrite(TRIG[i], LOW);
  }
#if TEMP_TYPE == 1
  dht.begin();
#endif
}

void loop() {
  long d[3] = {-1, -1, -1};
  for (int i = 0; i < NB_ULTRASONS; i++) { d[i] = mesurerUltrason(i); delay(15); }
  Serial.print("c=");  Serial.print(d[0]);
  Serial.print(" g="); Serial.print(d[1]);
  Serial.print(" d="); Serial.print(d[2]);

  if (GAZ_ACTIF && moduleBranche(BROCHE_GAZ)) {
    Serial.print(" gaz="); Serial.print(moyenneAnalogique(BROCHE_GAZ));
  }

#if TEMP_TYPE == 0
  if (TEMP_ACTIF && moduleBranche(BROCHE_TEMP)) {
    float t = lireTemperature();
    Serial.print(" tadc="); Serial.print(dernierAdcTemp);
    if (!isnan(t) && t > -20 && t < 80) { Serial.print(" temp="); Serial.print(t, 1); }
  }
#else
  if (TEMP_ACTIF) {
    float t = lireTemperature();
    if (!isnan(t)) { Serial.print(" temp="); Serial.print(t, 1); }
  }
#endif

  if (VAPEUR_ACTIF && moduleBranche(BROCHE_VAPEUR)) {
    Serial.print(" vapeur="); Serial.print(moyenneAnalogique(BROCHE_VAPEUR));
  }
  if (LUM_ACTIF && moduleBranche(BROCHE_LUM)) {
    Serial.print(" lum="); Serial.print(moyenneAnalogique(BROCHE_LUM));
  }
  Serial.println();
  delay(20);
}
