#include <ESC.h>

ESC ESC1 (9,1000,2000,500);  
ESC ESC2 (6,1000,2000,500);
ESC ESC3 (5,1000,2000,500);
ESC ESC4 (3,1000,2000,500);
int potValue;
int analog;

void setup() {
  ESC1.calib();  
  //ESC2.calib();
  //ESC3.calib();
  //ESC4.calib();
  ESC1.stop();
  //ESC2.stop();
  //ESC3.stop();
  //ESC4.stop();
  ESC1.arm();
  delay(5000);
}

void loop() {
  for (potValue = 1000; potValue <= 2000; potValue++) {
    ESC1.speed(analog);
    ESC2.speed(analog);
    ESC3.speed(analog);
    ESC4.speed(analog);
    delay(15);
  }
  delay(1000);
  for (potValue = 2000; potValue >= 1000; potValue--) {
    ESC1.speed(analog);
    ESC2.speed(analog);
    ESC3.speed(analog);
    ESC4.speed(analog);
    delay(15);
  }
  delay(5000);
}