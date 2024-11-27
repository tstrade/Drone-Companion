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

float dist;
float motorSpeed;

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);
int IRCode  = 0;

int i = 0;

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
  checkIRCode();
  Serial.println(IRCode);
  backRadar.write(trigBack);
  dist = calculateDistance(trigBack, echoBack);
  Serial.print("User dist (cm) is: ");
  Serial.println(dist);

  if (IRCode == 3) { // Start motors
    motorBackRight.write(0);
    motorFrontLeft.write(0);
    motorBackLeft.write(0);
    motorFrontRight.write(0);
  } else if (IRCode == 2){ // Stop motors
    motorBackRight.write(100);
    motorFrontLeft.write(100);
    motorBackLeft.write(100);
    motorFrontRight.write(100);
  } else if (IRCode == 0) { // Motors based on radars
    motorSpeed = map(dist, 0, 300, 50, 180);
    Serial.print("New speed is: ");
    Serial.println(motorSpeed);
    motorBackRight.write(motorSpeed);
    motorFrontLeft.write(motorSpeed);
    motorBackLeft.write(motorSpeed);
    motorFrontRight.write(motorSpeed);
  } else { // Idle speed
    motorBackRight.write(50);
    motorFrontLeft.write(50);
    motorBackLeft.write(50);
    motorFrontRight.write(50);
  }
}