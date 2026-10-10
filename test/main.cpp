#include <Wire.h>
#include <Arduino.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_CLOCK 100000  // 100kHz - mais confiável pra scan que clock alto

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1000);
  Serial.println("\nScanner I2C iniciado...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("\nScaneando...");
  Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");

  for (int row = 0; row < 8; row++) {
    Serial.print("0x");
    if (row < 16) Serial.print("0");
    Serial.print(row, HEX);
    Serial.print("0 ");

    for (int col = 0; col < 16; col++) {
      address = row * 16 + col;

      if (address < 1 || address > 126) {
        Serial.print(" . ");
        continue;
      }

      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0) {
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.print(" ");
        nDevices++;
      } else {
        Serial.print(" - ");
      }
    }
    Serial.println();
  }

  Serial.println();
  if (nDevices == 0) {
    Serial.println("Nenhum dispositivo encontrado\n");
  } else {
    Serial.print(nDevices);
    Serial.println(" dispositivo(s) encontrado(s)\n");
  }

  delay(2000); // reduzido de 5000 pra debugar mais rápido
}