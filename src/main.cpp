#include <Arduino.h>
#include <Wire.h>

const byte TCN75A_ADDR = 0x48; // adresse trouvee a la manipulation 1
const byte PTR_TEMP = 0x00;
const byte PTR_CONFIG = 0x01;
// bits 6-5 du registre CONFIG : 00=9 bits, 01=10 bits, 10=11 bits, 11=12 bits
const byte RESOLUTION[4] = {0b00000000, 0b00100000, 0b01000000, 0b01100000};
const char *NOM_RESOLUTION[4] = {"9 bits (0.5 degC)", "10 bits (0.25 degC)",
                                 "11 bits (0.125 degC)",
                                 "12 bits (0.0625 degC)"};
const uint16_t TCONV_TYPIQUE_MS[4] = {
    30, 60, 120, 240}; // fiche technique, tableau ci-dessus
void ecrireRegistre8(uint8_t pointeur, uint8_t valeur) {
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(pointeur);
  Wire.write(valeur);
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
}
void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    ecrireRegistre8(PTR_CONFIG, RESOLUTION[i]);
    delay(TCONV_TYPIQUE_MS[i] +
          50); // on laisse le temps a une conversion de se terminer
    Serial.print("Resolution ");
    Serial.print(NOM_RESOLUTION[i]);
    Serial.print(" -> T = ");
    Serial.print(lireTemperatureEnC(), 4);
    Serial.println(" degC");
    delay(3000);
  }
  Serial.println("---");
}