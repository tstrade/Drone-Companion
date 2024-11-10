#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp(10);


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100); // wait for native usb
  }

  bmp.begin();
}

void loop() {
  // Collect altitude data
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25) / 3.281); 
  Serial.println(" feet");

  Serial.println();
  delay(2000);
}

