#include <Servo.h>
#include <IRremote.h>

//define motor pins
#define BACKRIGHT 9
#define FRONTLEFT 3
#define BACKLEFT 6
#define FRONTRIGHT 5

// Define the servo motors (ESCs)
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);
int IRCode  = 0;

int minMotorSpeed = 1000;  // Minimum throttle
int maxMotorSpeed = 2000;  // Maximum throttle

void armESC() {
    // Set high throttle
    delay(2000);
    motorBackRight.write(maxMotorSpeed);
    motorFrontLeft.write(maxMotorSpeed);
    motorBackLeft.write(maxMotorSpeed);
    motorFrontRight.write(maxMotorSpeed);
    delay(5000);
    // Set low throttle
    motorBackRight.write(minMotorSpeed);
    motorFrontLeft.write(minMotorSpeed);
    motorBackLeft.write(minMotorSpeed);
    motorFrontRight.write(minMotorSpeed);
    delay(3000);   
    // Arm ESCs
    motorBackRight.write(500);
    motorFrontLeft.write(500);
    motorBackLeft.write(500);
    motorFrontRight.write(500);
    delay(1500);
}

void setup() {
  // Attach ESCs to the appropriate pins
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(5); // Pin 5 for front right motor
  motorBackLeft.attach(6);   // Pin 6 for back left motor
  motorBackRight.attach(9);  // Pin 9 for back right motor
  //calibration and arming sequence is time sensitive
  armESC();
  //setup IR
  recieverIR.enableIRIn();
  Serial.begin(9600);
}

int checkIRCode(){
  if (recieverIR.decode()) { 
    IRCode = recieverIR.decodedIRData.command; 
  }
  recieverIR.resume();
  return IRCode;
}

void loop() {
  // Start motors
  checkIRCode();
  Serial.println(IRCode);
  if (IRCode == 3) {
    motorBackRight.write(minMotorSpeed);
    motorFrontLeft.write(minMotorSpeed);
    motorBackLeft.write(minMotorSpeed);
    motorFrontRight.write(minMotorSpeed);
  } else if (IRCode == 2){
    motorBackRight.write(350);
    motorFrontLeft.write(350);
    motorBackLeft.write(350);
    motorFrontRight.write(350);
  }
}

