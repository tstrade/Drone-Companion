#include <Servo.h>
#include <TimeLib.h>

Servo ESC1;   
Servo ESC2;
Servo ESC3;
Servo ESC4;
int analog;
int potValue;
time_t base;

void armESC(Servo ESC) {
    ESC.write(2000);        // start off setting max throttle
    delay(6000);            // give the ESC time to see it and make note
    ESC.write(1000);       // go to low-throttle
    delay(5000);            // this might need to be adjusted up/down not sure
    analog = map(0, 0, 1023, 0, 180);
    ESC.write(analog);
}

void setup() {
  base = now();
  // Attach the ESC on pin 9
  ESC1.attach(9);
  printf("Time: %d\n", base - now());
  //armESC(ESC1);
  ESC2.attach(3);
  printf("Time: %d\n", base - now());
  //armESC(ESC2);
  ESC3.attach(6);
  printf("Time: %d\n", base - now());
  //armESC(ESC3);
  ESC4.attach(5);
  printf("Time: %d\n", base - now());
  //armESC(ESC4);
  ESC1.write(500);
  ESC2.write(500);
  ESC3.write(500);
  ESC4.write(500);
  printf("Time: %d\n", base - now());
  delay(1000);
}

void loop() {
  printf("Time: %d\n", base - now());
  for (potValue = 0; potValue < 1024; potValue++) {
    analog = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC1.write(analog);    // Send the signal to the ESC
    ESC2.write(analog);
    ESC3.write(analog);
    ESC4.write(analog);
  }
  delay(3000);
  for (potValue = 1023; potValue > 0; potValue++) {
    analog = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC1.write(analog);    // Send the signal to the ESC
    ESC2.write(analog);
    ESC3.write(analog);
    ESC4.write(analog);
  }
  delay(10000);
}