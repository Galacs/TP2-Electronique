#include <Arduino.h>
#include <Wire.h>

const byte TCN75A_ADDR = 0x48;
const byte PTR_THYST = 0x02;
const byte PTR_TSET = 0x03;
int16_t lireRegistre16(uint8_t pointeur) {
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(pointeur);
  Wire.endTransmission(false); // repeated start : on garde le bus ouvert
  Wire.requestFrom(TCN75A_ADDR, (uint8_t)2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (int16_t)((msb << 8) | lsb);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  int16_t tset = lireRegistre16(PTR_TSET);
  int16_t thyst = lireRegistre16(PTR_THYST);
  Serial.print("TSET lu : 0x");
  Serial.print((uint16_t)tset, HEX);
  Serial.print(" (");
  Serial.print(tset / 256.0, 2);
  Serial.println(" degC)");
  Serial.print("THYST lu : 0x");
  Serial.print((uint16_t)thyst, HEX);
  Serial.print(" (");
  Serial.print(thyst / 256.0, 2);
  Serial.println(" degC)");
  if (tset == (int16_t)0x5000 && thyst == (int16_t)0x4B00) {
    Serial.println(
        "-> TCN75A identifie avec succes (valeurs de reset conformes) !");
  } else {
    Serial.println(
        "-> Valeurs inattendues : soit ce n’est pas un TCN75A, soit les");
    Serial.println(" registres TSET/THYST ont deja ete modifies par un "
                   "programme precedent.");
  }
}

void loop() {}