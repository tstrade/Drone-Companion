#include <Servo.h>

Servo ESC1;   
Servo ESC2;
Servo ESC3;
Servo ESC4;
int analog;
int potValue;
int active;


void setESCSpeed(int speed) {
    ESC1.write(speed);
    ESC2.write(speed);
    ESC3.write(speed);
    ESC4.write(speed);
}

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(9);
  ESC2.attach(3);
  ESC3.attach(6);
  ESC4.attach(5);
  setESCSpeed(500);
  active = 1;
  delay(1000);
}

void loop() {
  if (active) {
    for (potValue = 0; potValue < 1024; potValue++) {
      analog = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
      setESCSpeed(analog);
      delay(2000);
    }
    setESCSpeed(0);
    active = 0;
    delay(10000);
  } else {
      ESC1.detach(9);
      ESC2.detach(3);
      ESC3.detach(6);
      ESC4.detach(5);
  }
}
