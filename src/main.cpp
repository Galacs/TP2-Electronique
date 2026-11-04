#include <Arduino.h>
#include <Wire.h>

const byte TCN75A_ADDR = 0x48; // adresse trouvee a la manipulation 1
const byte PTR_TEMP =
    0x00; // pointeur du registre temperature (TA), lecture seule
int16_t lireRegistreTemperature() {
  // Etape 1 : on ecrit le pointeur pour selectionner le registre TA
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(PTR_TEMP);
  Wire.endTransmission(false); // repeated start : on garde le bus ouvert
  // Etape 2 : on lit les 2 octets du registre (MSB puis LSB)
  Wire.requestFrom(TCN75A_ADDR, (uint8_t)2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (msb << 8) | lsb;
  // TODO : reconstituer un entier SIGNE sur 16 bits a partir de msb et lsb
  // (indice : (msb << 8) | lsb, puis transtypage en int16_t)
  // TODO : RENVOYER (return ...;) cet entier signe -- la conversion en degC
  // se fera dans lireTemperatureEnC() ci-dessous
}
float lireTemperatureEnC() {
  return lireRegistreTemperature()/256.0;
  // TODO : diviser "brut" par 256.0 (2^8) pour obtenir la temperature en degC
  
  // (indice : le registre code la temperature en 1/256 degC, cf. partie 2.6
  // du polycopie -- ne pas oublier le ".0" pour forcer une division flottante)
  // et RENVOYER (return ...;) le resultat
}
void setup() {
  Wire.begin();
  Serial.begin(115200);
}
void loop() {
  float t = lireTemperatureEnC();
  Serial.print("Temperature = ");
  Serial.print(t, 4);

  Serial.println(" degC");
  delay(500);
}