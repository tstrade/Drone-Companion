#include <Servo.h>

#define   MotorPin    5    // whatever PWM capable pin you want to use
#define   PotPin     A0    // choose the analog input pin

Servo ESC;                // Servo object to act as brushless DC controller
int analog;
int potValue;

void armESC() {
    ESC.write(2000);        // start off setting max throttle
    delay(10000);            // give the ESC time to see it and make note
    ESC.write(1000);       // go to low-throttle
    delay(3000);            // this might need to be adjusted up/down not sure
    ESC.write (0);        // turn totally off
}

void setup() {
  // Attach the ESC on pin 9
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  armESC();
}

void loop() {
  for (potValue = 0; potValue < 1024; potValue++) {
    analog = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(analog);    // Send the signal to the ESC
  }
  for (potValue = 1024; potValue > 0; potValue++) {
    analog = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(analog);    // Send the signal to the ESC
  }
}