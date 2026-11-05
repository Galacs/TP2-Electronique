#include <Arduino.h>
#include <Wire.h>

const byte BMA220_ADDR = 0x0A;
// A REMPLACER par l’adresse trouvee au scan
uint8_t lireRegistre(uint8_t registre) {
  Wire.beginTransmission(BMA220_ADDR);
  Wire.write(registre);
  Wire.endTransmission(false);
  // repeated start : on garde le bus ouvert
  Wire.requestFrom(BMA220_ADDR, (uint8_t)1);
  return Wire.read();
}
void setup() {
  Wire.begin();
  Serial.begin(115200);
  uint8_t id = lireRegistre(0x00);
  Serial.print("chip_id lu : 0x");
  Serial.println(id, HEX);
  if (id == 0xDD) {
    Serial.println("-> BMA220 identifie avec succes !");
  } else {
    Serial.println("-> Valeur inattendue : verifiez l’adresse et le cablage.");
  }
}
void loop() {}