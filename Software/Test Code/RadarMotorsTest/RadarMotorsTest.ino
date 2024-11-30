#include <Servo.h>
#include <IRremote.h>
#include "pitches.h"

// buzzer pin and tone
#define BUZZER_PIN 8
const int alert_tone = NOTE_FS5;

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

int obstacleFlag = 0;
float prevSpeed = 0;
float motorSpeed, userDist;

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);
int IRCode  = 0;

int minMotorSpeed = 90;  // Minimum throttle
int maxMotorSpeed = 180;  // Maximum throttle

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
    motorBackRight.write(45);
    motorFrontLeft.write(45);
    motorBackLeft.write(45);
    motorFrontRight.write(45);
    delay(1500);
}

void setup() {
  Serial.begin(9600);
  // Attach ESCs
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(9); // Pin 5 for front right motor
  motorBackLeft.attach(5);   // Pin 6 for back left motor
  motorBackRight.attach(6);  // Pin 9 for back right motor

  // Calibration and Arming sequence
  //armESC();
  Serial.println("Plug in ESCs now - wait for confirmation beep");
  motorBackRight.write(minMotorSpeed);
  motorFrontLeft.write(minMotorSpeed);
  motorBackLeft.write(minMotorSpeed);
  motorFrontRight.write(minMotorSpeed);
  delay(3000);
  while (!Serial.available()) {}
  Serial.read();

  motorBackRight.write(minMotorSpeed);
  motorFrontLeft.write(minMotorSpeed);
  motorBackLeft.write(minMotorSpeed);
  motorFrontRight.write(minMotorSpeed);
  delay(3000);

  // Setup IR sensor
  recieverIR.enableIRIn();

  // Setup radars
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);

  // Setup buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

int checkIRCode(){
  if (recieverIR.decode()) { 
    IRCode = recieverIR.decodedIRData.command; 
  }
  recieverIR.resume();
  return IRCode;
}

float calculateDistance(int trig, int echo, int flag) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float dist = pulseIn(echo, HIGH) * 0.034/2;
  if (trig == trigFront && flag && dist < 15.0) {
    tone(BUZZER_PIN, alert_tone);
    return dist;
  } else {
    return dist;
  }
}

void loop() {
  // Receive input from remote
  checkIRCode();
  Serial.println(IRCode);

  // Check user distance to adjust motors
  backRadar.write(trigBack);
  userDist = calculateDistance(trigBack, echoBack, 0);
  Serial.print("User dist (cm) is: ");
  Serial.println(userDist);

  // Setup reference speed
  if (!prevSpeed) {
      prevSpeed = map(userDist, 0, 300, 50, 180);
  }

  // Detect obstacle distance to alert buzzer
  frontRadar.write(trigFront);
  int obstacleDist = calculateDistance(trigFront, echoFront, obstacleFlag);
  Serial.print("Obstacle distance is: ");
  Serial.println(obstacleDist);
  if (obstacleFlag) { delay(250); }
  noTone(BUZZER_PIN); 
  
  switch (IRCode) {
    case 3: // Stop motors
      obstacleFlag = 0;
      motorBackRight.write(0);
      motorFrontLeft.write(0);
      motorBackLeft.write(0);
      motorFrontRight.write(0);
      break;
    case 2: // Medium-high speed
      obstacleFlag = 0;
      motorBackRight.write(100);
      motorFrontLeft.write(100);
      motorBackLeft.write(100);
      motorFrontRight.write(100);
      break;
    case 0: // Speed & buzzer based on radars
      obstacleFlag = 1;
      motorSpeed = (2 * map(userDist, 0, 300, 50, 180) + 3 * prevSpeed) / 5;
      prevSpeed = motorSpeed;
      Serial.print("New speed is: ");
      Serial.println(motorSpeed);
      motorBackRight.write(motorSpeed);
      motorFrontLeft.write(motorSpeed);
      motorBackLeft.write(motorSpeed);
      motorFrontRight.write(motorSpeed);
      break;
    default: // Idle speed
      obstacleFlag = 0;
      motorBackRight.write(50);
      motorFrontLeft.write(50);
      motorBackLeft.write(50);
      motorFrontRight.write(50);
      break;
  }
}