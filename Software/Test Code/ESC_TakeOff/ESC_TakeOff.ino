#include <Servo.h>

#define ESC1_PIN 9
#define ESC2_PIN 3
#define ESC3_PIN 6
#define ESC4_PIN 5

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

void armESC() {
    Serial.print("PLUG IN NOW!\n");
    // Set high throttle
    delay(2000);
    ESC1.write(2000);
    ESC2.write(2000);
    ESC3.write(2000);
    ESC4.write(2000);
    delay(5000);
    // Set low throttle
    ESC1.write(1000);
    ESC2.write(1000);
    ESC3.write(1000);
    ESC4.write(1000);
    delay(3000);   
    // Arm ESCs
    ESC1.write(500);
    ESC2.write(500);
    ESC3.write(500);
    ESC4.write(500);
}

void setup() {
    Serial.begin(9600);
    // Attach the ESC on pin 9
    ESC1.attach(ESC1_PIN);
    // Attach the ESC on pin 3
    ESC2.attach(ESC2_PIN);
    // Attach the ESC on pin 6
    ESC3.attach(ESC3_PIN);
    // Attach the ESC on pin 5
    ESC4.attach(ESC4_PIN);
    armESC();
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
    delay(20000);
  } else {
      ESC1.detach();
      ESC2.detach();
      ESC3.detach();
      ESC4.detach();
  }
}
