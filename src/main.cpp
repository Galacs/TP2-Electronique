#include <Arduino.h>
#include <Wire.h>

// TP2
void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  int nbTrouves = 0;
  for (byte adresse = 1; adresse < 127; adresse++) {
    Wire.beginTransmission(adresse);
    byte erreur = Wire.endTransmission();
    if (erreur == 0) {
      Serial.print("Composant trouve a l’adresse 0x");
      if (adresse < 16)
        Serial.print("0");
      Serial.println(adresse, HEX);
      nbTrouves++;
    }
  }
  if (nbTrouves == 0) {
    Serial.println("Aucun composant trouve. Verifiez le cablage !");
  }
  Serial.println("---");
  delay(3000);
}