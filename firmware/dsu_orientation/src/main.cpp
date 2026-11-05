// Variante "pont DSU" du TP : sortie machine lisible a 50 Hz.
//
// Format d'une ligne (une par mesure) :
//     DSU,ax,ay,az,angle
// avec ax/ay/az en g et angle en degres (atan2(ax, az), comme dans le TP).
//
// A flasher avec :  pio run -e dsu -t upload
// Le sketch du TP (src/main.cpp) n'est pas modifie : on peut revenir dessus
// avec :            pio run -e uno -t upload
#include <Arduino.h>
#include <Wire.h>

namespace {

const byte BMA220_ADDR = 0x0A;  // adresse trouvee a la manipulation 1

const byte REG_RANGE = 0x22;    // registre de gamme
const byte RANGE_2G = 0x00;     // +/-2 g -> 16 LSB/g, 6 bits utiles
const uint8_t REG_AX = 0x04;
const uint8_t REG_AY = 0x06;
const uint8_t REG_AZ = 0x08;

const unsigned long PERIODE_MS = 20;  // 50 Hz

uint8_t lireRegistre(uint8_t registre) {
  Wire.beginTransmission(BMA220_ADDR);
  Wire.write(registre);
  Wire.endTransmission(false);
  Wire.requestFrom(BMA220_ADDR, (uint8_t)1);
  return Wire.read();
}

void ecrireRegistre(uint8_t registre, uint8_t valeur) {
  Wire.beginTransmission(BMA220_ADDR);
  Wire.write(registre);
  Wire.write(valeur);
  Wire.endTransmission();
}

// Valeur signee sur 6 bits (bits 7..2), en g.
float lireAxeEnG(uint8_t registreAxe) {
  int8_t brut = (int8_t)lireRegistre(registreAxe);
  return (brut >> 2) / 16.0f;
}

}  // namespace

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Les registres du BMA220 sont volatils mais ne sont pas reinitialises
  // par un televersement : on impose explicitement la gamme +/-2 g.
  ecrireRegistre(REG_RANGE, RANGE_2G);

  delay(50);
  Serial.println(F("DSU orientation 50 Hz"));
}

void loop() {
  float ax = lireAxeEnG(REG_AX);
  float ay = lireAxeEnG(REG_AY);
  float az = lireAxeEnG(REG_AZ);
  float angle = atan2(ax, az) * 180.0f / PI;

  // Ligne machine : c'est celle que lit dsu_bridge.py.
  Serial.print(F("DSU,"));
  Serial.print(ax, 4);
  Serial.print(',');
  Serial.print(ay, 4);
  Serial.print(',');
  Serial.print(az, 4);
  Serial.print(',');
  Serial.println(angle, 2);

  delay(PERIODE_MS);
}
