//this file handles how the motors interact with the values recieved from the Radar sensors. 
//and the buzzer too because why not.
#include <Servo.h>
#include <pitches.h>

//define motor pins
#define BACKRIGHT 9
#define FRONTLEFT 3
#define BACKLEFT 6
#define FRONTRIGHT 5

//define radar pins
#define trigFront 16
#define echoFront 17
#define trigBack 14
#define echoBack 15

//define buzzer pin
#define BUZZER_PIN 8

// Define the servo motors (ESCs)
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

//define radar sensors using servo object
Servo frontRadar;
Servo backRadar;

//define a list of tones the buzzer can produce
const int gameTones[] = { NOTE_G4, NOTE_A4, NOTE_E4, NOTE_D4};

int minMotorSpeed = 1000;  // Minimum throttle for motors
int maxMotorSpeed = 2000;  // Maximum throttle for motors
#define ARMVAL 500

const float preferredDistance = 91.44;

//calibrate and arm ESCs/Motors
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
    motorBackRight.write(ARMVAL);
    motorFrontLeft.write(ARMVAL);
    motorBackLeft.write(ARMVAL);
    motorFrontRight.write(ARMVAL);
}

//calculate distance from duration readings of the radar sensors
int calculateDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.034/2);
}

void setup() {
  // Attach ESCs to the appropriate pins
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(5); // Pin 5 for front right motor
  motorBackLeft.attach(6);   // Pin 6 for back left motor
  motorBackRight.attach(9);  // Pin 9 for back right motor
  //calibration and arming sequence is time sensitive
  armESC();

 //setup radars output and input reading
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  digitalWrite(trigFront, LOW);

  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);
  digitalWrite(trigBack, LOW);

  //setup buzzer pin output reading
  pinMode(BUZZER_PIN, OUTPUT);

  delay(2000);
}

//increases speed of the two back motors to make the drone move forward
void moveForwards(float updatedSpeed){
  motorBackLeft.write(updatedSpeed);
  motorBackRight.write(updatedSpeed);
}

//increases speed of the two fron motors to make the drone move backwards
void moveBackwards(float updatedSpeed){
  motorFrontLeft.write(updatedSpeed);
  motorFrontRight.write(updatedSpeed);
}

//handles both radar sensors
void follow(float motorSpeed){
  float userDistance = calculateDistance(trigBack, echoBack);
  float obstacleDistance = calculateDistance(trigFront, echoFront);
  float userDistanceDifference;
  float obstacleDistanceDifference;
  //drone will have normal function if there is no obstacle in the way
  while(obstacleDistance > preferredDistance){
    //turn off buzzer
    noTone(BUZZER_PIN);

    //if the user is far from the drone, the drone will move closer to the user, about 3 feet away.
    while(userDistance > preferredDistance){
      userDistance = calculateDistance(trigBack, echoBack);
      userDistanceDifference = userDistance - preferredDistance;
      motorSpeed = map(userDistanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
      motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds
      moveBackwards(motorSpeed);
    }
    //if user is too close to the drone, the drone will move about 3 feet away form the user
    while(userDistance < preferredDistance){
      userDistance = calculateDistance(trigBack, echoBack);
      userDistanceDifference = preferredDistance - userDistance;
      motorSpeed = map(userDistanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
      motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds
      moveForwards(motorSpeed);
    }
  }
  //if the drone detects an obstacle. the drone will not respond to any movement that will cause it to mvoe closer to the obstacle
  while(obstacleDistance <= preferredDistance){
    //turn on buzzer tone
    tone(BUZZER_PIN, gameTones[0]);

    //Drone will only move if user moves away from drone.
    if(userDistance > preferredDistance){
      userDistance = calculateDistance(trigBack, echoBack);
      userDistanceDifference = userDistance - preferredDistance;
      motorSpeed = map(userDistanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
      motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds
      moveBackwards(motorSpeed);
      break;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  follow(minMotorSpeed);
}
