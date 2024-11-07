#include <Servo.h>

Servo ESC1;   
Servo ESC2;
Servo ESC3;
Servo ESC4;
int analog;
int potValue;

void armESC() {
    ESC1.write(2000);        // start off setting max throttle
    ESC2.write(2000);
    ESC3.write(2000);
    ESC4.write(2000);
    delay(8000);            // give the ESC time to see it and make note
    ESC1.write(1000);       // go to low-throttle
    ESC2.write(1000);
    ESC3.write(1000);
    ESC4.write(1000);
    delay(3000);            // this might need to be adjusted up/down not sure
    ESC1.write (500);        // turn totally off
    ESC2.write (500);
    ESC3.write (500);
    ESC4.write (500);
}

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(9);
  ESC2.attach(3);
  ESC3.attach(6);
  ESC4.attach(5);
  armESC();
  delay(1000);
}

void loop() {
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
