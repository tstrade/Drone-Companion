#include <Servo.h>

Servo motorFrontLeft;   
Servo motorFrontRight;
Servo motorBackLeft;
Servo motorBackRight;

const int minMotorSpeed = 1000;
const int maxMotorSpeed = 2000;

void armESC() {
    // Set high throttle
    motorBackRight.write(maxMotorSpeed);
    motorFrontLeft.write(maxMotorSpeed);
    motorBackLeft.write(maxMotorSpeed);
    motorFrontRight.write(maxMotorSpeed);

    Serial.println("Connect ESCs - wait for two beeps to confirm high throttle");
    
    while (!Serial.available()) {}
    Serial.read();

    Serial.println("\nSetting low throttale - wait for several beeps then long beep, then push any key");
    // Set low throttle
    motorBackRight.write(minMotorSpeed);
    motorFrontLeft.write(minMotorSpeed);
    motorBackLeft.write(minMotorSpeed);
    motorFrontRight.write(minMotorSpeed);

    delay(2500);
    Serial.println("\nESCs can now be unplugged.");
}

void setup() {
  Serial.begin(9600);
  // Attach ESCs
  motorFrontLeft.attach(3);
  motorFrontRight.attach(9);
  motorBackLeft.attach(5);
  motorBackRight.attach(6);
}

void loop() {
  armESC();
  delay(10000);
}
