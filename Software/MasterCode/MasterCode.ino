#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> 
#include <IRremote.h>
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

//define pi
#define PI 3.14127

// Define the servo motors (ESCs)
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

// Create an instance of the MPU6050 object
MPU6050 mpu;

// Create an instance of the BMP280 object
Adafruit_BMP280 bmp(10);

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);

//define radar sensors using servo object
Servo frontRadar;
Servo backRadar;

//define a list of tones the buzzer can produce
const int gameTones[] = { NOTE_G4, NOTE_A4, NOTE_E4, NOTE_D4};

// Initial motor speed (this will be modified later)
int minMotorSpeed = 1000;  // Base motor speed (minimum throttle)
int maxMotorSpeed = 2000;  // Maximum throttle for motors

float altitudeThreshold = 5.0;

const float stdAirPressure = 1013.25;

const float preferredDistance = 100.0; // Preferred distance in cm
const float distanceTolerance = 10.0; // Tolerance of +/- 10cm

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
}

void setup() {
  // Attach ESCs to the appropriate pins
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(5); // Pin 5 for front right motor
  motorBackLeft.attach(6);   // Pin 6 for back left motor
  motorBackRight.attach(9);  // Pin 9 for back right motor
  //calibration and arming sequence is time sensitive
  armESC();

  // Initialize and calibrate the MPU6050
  Wire.begin();
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    delay(500);
  }
  mpu.calibrateGyro();

  // Initialize BMP280
  bmp.begin();

  //setup IR
  recieverIR.enableIRIn();

  //setup radars output and input reading
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  digitalWrite(trigFront, LOW);

  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);
  digitalWrite(trigBack, LOW);

  //setup buzzer pin output reading
  pinMode(BUZZER_PIN, OUTPUT);
  
  /* Set initial motor speeds (idle speed for ESCs)
  motorFrontLeft.writeMicroseconds(minMotorSpeed);
  motorFrontRight.writeMicroseconds(minMotorSpeed);
  motorBackLeft.writeMicroseconds(minMotorSpeed);
  motorBackRight.writeMicroseconds(minMotorSpeed);
  */
  delay(2000);
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

void moveForwards(float updatedSpeed) {
  motorBackLeft.write(updatedSpeed);
  motorBackRight.write(updatedSpeed);
}

void moveBackwards(float updatedSpeed) {
  motorFrontLeft.write(updatedSpeed);
  motorFrontRight.write(updatedSpeed);
}

// Follow user function with the back radar
void follow(float motorSpeed) {
  float userDistance = calculateDistance(trigBack, echoBack);
  float distanceDifference;
  
  // Move towards user gradually 
  while(userDistance > preferredDistance){
    userDistance = calculateDistance(trigBack, echoBack);
    distanceDifference = userDistance - preferredDistance;

    motorSpeed = map(distanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds

    moveBackwards(motorSpeed);
    stabilizeDrone(motorSpeed);
    delay(20);
  }

  // Move away from user gradually
  while(userDistance < preferredDistance){
    userDistance = calculateDistance(trigBack, echoBack);
    distanceDifference = preferredDistance - userDistance;

    motorSpeed = map(distanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds

    moveForwards(motorSpeed);
    stabilizeDrone(motorSpeed);
    delay(20);
  }
  //!!add staalize function!!//
  stabilizeDrone(minMotorSpeed + 500);
  delay(100);
}

// Stabilize the drone's roll and pitch
void stabilizeDrone(float motorSpeed) {
    Vector ACCEL = mpu.readNormalizeAccel();
    float ax = ACCEL.XAxis, ay = ACCEL.YAxis, az = ACCEL.ZAxis;
    Vector GYRO = mpu.readNormalizeGyro();
    float gx = GYRO.XAxis, gy = GYRO.YAxis;

    // Calculate angles
    float roll = atan2(ay, az) * 180.0 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Proportional control for roll and pitch stabilization
    float rollCorrection = 0.1 * roll;
    float pitchCorrection = 0.1 * pitch;

    // Adjust motor speeds for stabilization
    int speedFrontLeft = motorSpeed + rollCorrection - pitchCorrection;
    int speedFrontRight = motorSpeed - rollCorrection - pitchCorrection;
    int speedBackLeft = motorSpeed + rollCorrection + pitchCorrection;
    int speedBackRight = motorSpeed - rollCorrection + pitchCorrection;

    // Apply speed corrections within bounds
    motorFrontLeft.writeMicroseconds(constrain(speedFrontLeft, minMotorSpeed, maxMotorSpeed));
    motorFrontRight.writeMicroseconds(constrain(speedFrontRight, minMotorSpeed, maxMotorSpeed));
    motorBackLeft.writeMicroseconds(constrain(speedBackLeft, minMotorSpeed, maxMotorSpeed));
    motorBackRight.writeMicroseconds(constrain(speedBackRight, minMotorSpeed, maxMotorSpeed));
}

// Hover in place for a set time
void hoverForTime(unsigned long duration, float motorSpeed) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        stabilizeDrone(motorSpeed);
        delay(20);  // Adjust delay as needed for smoother control loop
    }
}

// Takeoff sequence: gradually increase motor speeds
void takeOff(float initialAltitude, float targetAltitude) {
  int motorSpeed = minMotorSpeed;
  float altitude = bmp.readAltitude(stdAirPressure);

  while (altitude < initialAltitude + targetAltitude) {
    altitude = bmp.readAltitude(stdAirPressure);  // Read current altitude

    motorSpeed = map(altitude - initialAltitude, 0, targetAltitude, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds

    // Apply motor speed and stabilization to ESCs
    stabilizeDrone(motorSpeed);
    
    delay(50);  // Adjust delay for smoother takeoff
  }
}

// Landing sequence: gradually decrease motor speeds
void land(float initialAltitude) {
  int motorSpeed = maxMotorSpeed;
  float altitude;

  while (bmp.readAltitude(stdAirPressure) > initialAltitude + 10) {
    altitude = bmp.readAltitude(stdAirPressure);
    motorSpeed = constrain(map(altitude - initialAltitude, 0, 50, maxMotorSpeed, minMotorSpeed), minMotorSpeed, maxMotorSpeed);
       
    stabilizeDrone(motorSpeed);
    delay(50);
  }
}

void loop() {
  static float initialAltitude = bmp.readAltitude(stdAirPressure);
  static float targetAltitude = 100.0;  // Target altitude (in centimeters -> appox. 3 ft.) 

  if (recieverIR.decode()) {
    if (recieverIR.decodedIRData.command == 3) {
      takeOff(initialAltitude, targetAltitude);
      hoverForTime(15000, minMotorSpeed + 500);
    } else if (recieverIR.decodedIRData.command == 4) {
      land(initialAltitude);
    }
    recieverIR.resume();
  }
}
