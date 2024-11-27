#include <Servo.h>
#include <IRremote.h>

// motor pins
#define BACKRIGHT 9
#define FRONTLEFT 3
#define BACKLEFT 6
#define FRONTRIGHT 5

// radar pins
#define trigFront 14
#define echoFront 15
#define trigBack 4
#define echoBack 2

// Inititialize radars and motors in servo objects
Servo frontRadar, backRadar;
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);
int IRCode  = 0;

int i = 0;

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
  //setup radars pinout and input reading
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);

  Serial.begin(9600);
}

int checkIRCode(){
  if (recieverIR.decode()) { 
    IRCode = recieverIR.decodedIRData.command; 
  }
  recieverIR.resume();
  return IRCode;
}

float calculateDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH) * 0.034/2;
}

void loop() {
  float dist;
  int motorSpeed;
  checkIRCode();
  Serial.println(IRCode);
  // Start motors
  if (IRCode == 3) {
    for (int i; i < 100; i++) {
      backRadar.write(i);
      dist = calculateDistance(trigBack, echoBack);
      motorSpeed = map(motorSpeed, 0, 300, 50, 360);
      Serial.print("New speed is: ");
      Serial.println(motorSpeed);
      motorBackRight.write(motorSpeed);
      motorFrontLeft.write(motorSpeed);
      motorBackLeft.write(motorSpeed);
      motorFrontRight.write(motorSpeed);
    }
  // Stop motors
  } else if (IRCode == 2){
    motorBackRight.write(50);
    motorFrontLeft.write(50);
    motorBackLeft.write(50);
    motorFrontRight.write(50);
  } else {
    motorBackRight.write(180);
    motorFrontLeft.write(180);
    motorBackLeft.write(180);
    motorFrontRight.write(180);
  }
}