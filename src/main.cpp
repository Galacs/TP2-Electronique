#include <Arduino.h>
#include <Wire.h>

const byte BMA220_ADDR = 0x0A;
// adresse trouvee a la manipulation 1
const byte RANGE_00_2G = 0;
// range = 00 : gamme +/-2g (registre 0x22)
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
float lireAxeEnG(uint8_t registreAxe) {
  uint8_t brut = lireRegistre(registreAxe);
  int8_t valeurSignee = (int8_t)brut;
  return (valeurSignee >> 2)/16.0;
  // TODO : convertir "brut" (uint8_t, 8 bits) en entier SIGNE sur 8 bits
  // (indice : un simple cast (int8_t) suffit)
  // TODO : decaler ce nombre signe de 2 bits vers la droite (>> 2)
  // pour obtenir la valeur utile sur 6 bits (entre -32 et +31)
  // TODO : diviser par la sensibilite correspondant a RANGE_00_2G
  // (16.0 LSB/g pour la gamme +/-2g, cf. partie "Registres utiles")
  // et RENVOYER le resultat (instruction "return ...;")
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  // Les registres du BMA220 sont volatils mais NE sont PAS reinitialises
  // quand on televerse un programme (le module reste alimente) : on impose
  // donc explicitement la gamme +/-2g (registre 0x22, range = 00), cf. D10.
  ecrireRegistre(0x22, RANGE_00_2G);
}

void loop() {
  float ax = lireAxeEnG(0x04);
  float ay = lireAxeEnG(0x06);
  float az = lireAxeEnG(0x08);
  Serial.print("x = ");
  Serial.print(ax, 2);
  Serial.print(" gy = ");
  Serial.print(ay, 2);
  Serial.print(" gz = ");
  Serial.print(az, 2);
  Serial.println(" g");
  delay(200);
}