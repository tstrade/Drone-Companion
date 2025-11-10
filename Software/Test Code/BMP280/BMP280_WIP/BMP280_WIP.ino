#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp(10);
int ground;
const float airPressure = 1013.25;


void 
setup() {
  Serial.begin(9600);
  bmp.begin();
  ground = bmp.readAltitude(airPressure);
  Serial.println(ground);
}


void 
loop() {
  // Collect altitude data
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(airPressure) - ground); 
  Serial.println(F(" meters"));

  Serial.println();
  delay(2000);
}

