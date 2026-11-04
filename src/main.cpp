#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const byte TCN75A_ADDR = 0x48; // adresse trouvee a la manipulation 1
const byte PTR_TEMP = 0x00;
const byte PTR_CONFIG = 0x01;
const byte PTR_THYST = 0x02;
const byte PTR_TSET = 0x03;
const byte PIN_ALERT = 2; // ALERT du module relie a D2
// A ADAPTER a la temperature ambiante (comme en manipulation 4)
const float TEMPERATURE_TSET = 27.0;
const float TEMPERATURE_THYST = 25.0;
// (a) 0x02 : mode interruption, 9 bits, fault queue 1, ALERT active basse
// (b) remplacer par la valeur calculee en Q20 (c)
const byte CONFIG_VALEUR = 0x02;
void ecrireRegistre8(uint8_t pointeur, uint8_t valeur) {
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(pointeur);
  Wire.write(valeur);
  Wire.endTransmission();
}
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
  pinMode(PIN_ALERT, INPUT_PULLUP);
  ecrireRegistre8(PTR_CONFIG, CONFIG_VALEUR);
  ecrireRegistre16(PTR_TSET, TEMPERATURE_TSET);
  ecrireRegistre16(PTR_THYST, TEMPERATURE_THYST);
  Serial.print("CONFIG = 0x");
  Serial.println(CONFIG_VALEUR, HEX);
}

void loop() {
  // On lit ALERT AVANT toute lecture de registre (cf. Q21)
  bool alerte = (digitalRead(PIN_ALERT) == LOW);
  float t = lireTemperatureEnC();
  Serial.print(millis());
  Serial.print(" ms T = ");
  Serial.print(t, 4);
  Serial.print(" degC ALERT = ");
  Serial.println(alerte ? "ACTIVE !" : "inactive");
  delay(100);
}