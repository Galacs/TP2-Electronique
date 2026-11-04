#include <Arduino.h>
#include <Wire.h>

const byte TCN75A_ADDR = 0x48; // adresse trouvee a la manipulation 1
const byte PTR_TEMP = 0x00;
const byte PTR_CONFIG = 0x01;
void ecrireConfig(uint8_t valeur) {
  Wire.beginTransmission(TCN75A_ADDR);
  Wire.write(PTR_CONFIG);
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
  // 1) Mode continu (reglage par defaut) : conversion permanente en
  // arriere-plan
  ecrireConfig(0b00000000);
  delay(300);
  Serial.print("Mode continu : T = ");
  Serial.print(lireTemperatureEnC(), 3);
  Serial.println(" degC (~200 uA typique, fiche technique)");
  // 2) Mise en veille (shutdown, bit 0 = 1) : conversions arretees, T figee
  ecrireConfig(0b00000001);
  delay(500);
  Serial.print("Apres 0.5 s en veille : T = ");
  Serial.print(lireTemperatureEnC(), 3);
  Serial.println(" degC (valeur gelee ; ~0.1 a 2 uA typ./max en veille)");
  // 3) Mesure "one-shot" : une seule conversion sur demande, puis retour
  // automatique en veille (il faut deja etre en veille, bit 0 reste a 1,
  // et on positionne en plus le bit ONE-SHOT, bit 7)
  ecrireConfig(0b10000001);
  delay(50); // conversion 9 bits : ~30 ms typique (partie 4.5)
  Serial.print("Apres 1 mesure one-shot : T = ");
  Serial.print(lireTemperatureEnC(), 3);
  Serial.println(" degC");
  Serial.println("---");
  delay(3000);
}