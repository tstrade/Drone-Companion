#include <ESC.h>

ESC ESC1 (9,1000,2000,500);
// ESC ESC2 (6,1000,2000,500);  
// ESC ESC3 (5,1000,2000,500);
// ESC ESC4 (3,1000,2000,500);
// //int potValue;
// //int analog;

void setup() {
//   Serial.begin(9600);

//   Serial.println("calibrating:");

  ESC1.setCalibrationDelay(2000);
  ESC1.calib(); 
  ESC1.stop();
  Serial.println("ESC1 done");

  ESC2.setCalibrationDelay(3000);
  ESC2.calib();
  ESC2.stop();
  Serial.println("ESC2 done");

  ESC3.setCalibrationDelay(3000);
  ESC3.calib();
  ESC3.stop();
  Serial.println("ESC3 done");

  ESC4.setCalibrationDelay(3000);
  ESC4.calib();
  ESC4.stop();
  Serial.println("ESC4 done");

  Serial.println("}all ESCs done");

  Serial.println("Arming ESCs{");


  ESC1.arm();
  ESC1.stop();
  delay(1000);
  Serial.println("ESC1 Armed.");

  ESC2.arm();
  ESC2.stop();
  delay(1000);
  Serial.println("ESC2 Armed.");

  ESC3.arm();
  ESC3.stop();
  delay(1000);
  Serial.println("ESC3 Armed.");

  ESC4.arm();
  ESC4.stop();
  delay(1000);
  Serial.println("ESC4 Armed.");

  Serial.println("}All ESCs Armed");
}

void loop() {
 
  ESC1.speed(1500);
  ESC1.stop();
  delay(5000);
  ESC2.speed(1500);
  ESC2.stop();
  delay(5000);
  ESC3.speed(1500);
  ESC3.stop();
  delay(5000);
  ESC4.speed(1500);
  ESC4.stop();
  delay(5000);
  
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