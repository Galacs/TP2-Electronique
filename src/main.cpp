#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const byte TCN75A_ADDR = 0x48; // adresse trouvee a la manipulation 1
const byte PTR_TEMP = 0x00;
const byte PTR_CONFIG = 0x01;
const byte PTR_THYST = 0x02;

const byte PTR_TSET = 0x03;
const byte PIN_ALERT = 2; // ALERT du module relie a la broche D2 de l’Arduino
// A ADAPTER a la temperature ambiante mesuree a la manipulation 2 (visez un
// TSET environ 2 a 3 degC au-dessus de l’ambiante, pour pouvoir le declencher
// facilement avec la chaleur des doigts) :
const float TEMPERATURE_TSET = 26.0; // seuil haut : declenchement de l’alerte
const float TEMPERATURE_THYST =
    25.0; // seuil bas : arret de l’alerte (hysteresis)
void ecrireRegistre16(uint8_t pointeur, float temperatureC) {
  int16_t code = (int16_t)round(temperatureC * 256.0);
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(pointeur);
  Wire.write((code >> 8) & 0xFF); // MSB
  Wire.write(code & 0xFF);        // LSB
  Wire.endTransmission();
}
float lireTemperatureEnC() {
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(PTR_TEMP);
  Wire.endTransmission(false);
  Wire.requestFrom(TCN75A_ADDR, (uint8_t)2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  int16_t brut = (int16_t)((msb << 8) | lsb);
  return brut / 256.0;
}
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(
      PIN_ALERT,
      INPUT_PULLUP); // ALERT est une sortie a collecteur ouvert, active basse
  // Le registre CONFIG est laisse a sa valeur par defaut (0x00) : mode
  // comparateur, polarite active-basse, fault queue = 1 (cf. partie 2.5).
  // Seuls TSET et THYST doivent etre ecrits.
  ecrireRegistre16(PTR_TSET, TEMPERATURE_TSET);
  ecrireRegistre16(PTR_THYST, TEMPERATURE_THYST);
  Serial.print("TSET regle a ");
  Serial.print(TEMPERATURE_TSET, 1);
  Serial.print(" degC, THYST regle a ");
  Serial.print(TEMPERATURE_THYST, 1);
  Serial.println(" degC.");
}
void loop() {
  float t = lireTemperatureEnC();
  bool alerte = (digitalRead(PIN_ALERT) == LOW); // active basse (defaut usine)
  Serial.print("T = ");
  Serial.print(t, 2);
  Serial.print(" degC ALERT = ");
  Serial.println(alerte ? "ACTIVE !" : "inactive");
  delay(300);
}