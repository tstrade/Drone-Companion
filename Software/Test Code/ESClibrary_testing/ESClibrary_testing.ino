#include <ESC.h>

ESC ESC1 (9,1000,2000,500);
ESC ESC2 (6,1000,2000,500);  
ESC ESC3 (5,1000,2000,500);
ESC ESC4 (3,1000,2000,500);
// //int potValue;
// //int analog;

void setup() {
////////////////////////////////calib()////////////////////////////////////
  // //calibrate
  // ESC1.calib();
  // ESC1.stop();
  // delay(1000);

  // ESC2.calib();
  // ESC2.stop();
  // delay(8000);

  // ESC3.calib();
  // ESC3.stop();
  // delay(1000);

  // ESC4.calib();
  // ESC4.stop();
  // delay(1000);

  // //arm
  // ESC1.arm();
  // delay(1000);

  // ESC2.arm();
  // delay(1000);

  // ESC3.arm();
  // delay(5000);

  // ESC4.arm();
  // delay(5000);
//////////////////////////////////////////////////////////////////////////

////////////////////////////////Manual////////////////////////////////////
  //calibrate
  ESC1.speed(2000);
  ESC1.stop();
  delay(1000);
  ESC1.speed(1000);
  ESC1.stop();
  delay(1000);

  ESC2.speed(2000);
  ESC2.stop();
  delay(1000);
  ESC2.speed(1000);
  ESC2.stop();
  delay(1000);

  ESC3.speed(2000);
  ESC3.stop();
  delay(1000);
  ESC3.speed(1000);
  ESC3.stop();
  delay(1000);

  ESC4.speed(2000);
  ESC4.stop();
  delay(1000);
  ESC4.speed(1000);
  ESC4.stop();
  delay(1000);

  //arm
  ESC1.speed(500);
  ESC1.stop();
  delay(1000);

  ESC2.speed(500);
  ESC2.stop();
  delay(1000);

  ESC3.speed(500);
  ESC3.stop();
  delay(1000);

  ESC4.speed(500);
  ESC4.stop();
  delay(1000);
//////////////////////////////////////////////////////////////////////////
}

void loop() {
 
  ESC1.speed(1500);
  delay(1000);
  ESC1.stop();
  delay(1000);
  ESC2.speed(1500);
  delay(1000);
  ESC2.stop();
  delay(1000);
  ESC3.speed(1500);
  delay(1000);
  ESC3.stop();
  delay(1000);
  ESC4.speed(1500);
  delay(1000);
  ESC4.stop();
  delay(1000)
  
  // delay(1000);
  // for (potValue = 1000; potValue <= 2000; potValue++) {
  //   ESC1.speed(analog);
  //   // ESC2.speed(analog);
  //   // ESC3.speed(analog);
  //   // ESC4.speed(analog);
  //   delay(15);
  // }
  // delay(1000);
  // for (potValue = 2000; potValue >= 1000; potValue--) {
  //   ESC1.speed(analog);
  //   // ESC2.speed(analog);
  //   // ESC3.speed(analog);
  //   // ESC4.speed(analog);
  //   delay(15);
  // }
  // delay(5000);
}