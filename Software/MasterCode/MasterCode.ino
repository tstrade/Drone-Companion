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

// Complementary filter constants
float alpha = 0.98;  // Filter constant (tune as needed)
float dt = 0.01; // Time step

// Initial motor speed (this will be modified later)
int minMotorSpeed = 1000;  // Base motor speed (minimum throttle)
int maxMotorSpeed = 2000;  // Maximum throttle for motors

float altitudeThreshold = 5.0;

const float stdAirPressure = 1013.25;

const float preferredDistance = 91.44

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

//follow function not including functionality with the front: just follows user
void follow(float motorSpeed){
  float userDistance = calculateDistance(trigBack, echoBack);
  float distanceDifference
  while(userDistance > preferredDistance){
    userDistance = calculateDistance(trigBack, echoBack);
    distanceDifference = userDistance - preferredDistance;
    motorSpeed = map(distanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds
    moveBackwards(motorspeed)
  }
  while(userDistance < preferredDistance){
    userDistance = calculateDistance(trigBack, echoBack);
    distanceDifference = preferredDistance - userDistance;
    motorSpeed = map(distanceDifference, 0, preferredDistance, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds
    moveForwards(motorspeed)
  }
  //!!add staalize function!!//
}

void moveForward(updatedSpeed){
  motorBackLeft.write(updatedSpeed);
  motorBackRight.write(updatedSpeed);
}

void moveBackwards(updatedSpeed){
  motorFrontLeft.write(updatedSpeed);
  motorFrontRight.write(updatedSpeed);
}

// Takeoff sequence: gradually increase motor speeds
void takeoff(float initialAltitude, float targetAltitude) {
  int motorSpeed = minMotorSpeed;
  float altitude = bmp.readAltitude(stdAirPressure);
  float altitudeDifference;

  while (altitude < initialAltitude + targetAltitude) {
    altitude = bmp.readAltitude(stdAirPressure);  // Read current altitude
    altitudeDifference = altitude - initialAltitude;

    motorSpeed = map(altitudeDifference, 0, targetAltitude, minMotorSpeed, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);  // Ensure speed is within bounds

    // Apply motor speed to ESCs
    motorFrontLeft.writeMicroseconds(motorSpeed);
    motorFrontRight.writeMicroseconds(motorSpeed);
    motorBackLeft.writeMicroseconds(motorSpeed);
    motorBackRight.writeMicroseconds(motorSpeed);
    
    delay(50);  // Adjust delay for smoother takeoff
  }
}

void updateAngles(float ax, float ay, float az, float gx, float gy, float gz, float roll, float pitch, float rollAcc, float pitchAcc) {
  // Convert gyroscope data to degrees per second
  gx *= 180.0 / PI;  // Gyroscope x-axis rate in degrees/sec
  gy *= 180.0 / PI;  // Gyroscope y-axis rate in degrees/sec
  gz *= 180.0 / PI;  // Gyroscope z-axis rate in degrees/sec

  // Integrate the gyroscope readings to get roll and pitch
  roll += gx * dt;  // dt is the time step (e.g., 0.01 seconds)
  pitch += gy * dt;

  // Apply the complementary filter (weigh accelerometer and gyroscope)
  roll = alpha * (roll + gx * dt) + (1 - alpha) * rollAcc;
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitchAcc;

  // Update accelerometer-based angles
  rollAcc = atan2(ay, az) * 180.0 / PI;
  pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// Hover and stabilize the drone
void stabilize(float initialAltitude, float targetAltitude, float motorSpeed, unsigned long hoverStartTime, float lastAltitude) {
  // Read normalized accelerometer and gyroscope data
  Vector ACCEL = mpu.readNormalizeAccel();
  float ax = ACCEL.XAxis, ay = ACCEL.YAxis, az = ACCEL.ZAxis;
  Vector GYRO = mpu.readNormalizeGyro();
  float gx = GYRO.XAxis, gy = GYRO.YAxis, gz = GYRO.ZAxis;

  // Update roll and pitch angles using the complementary filer
  float roll = 0.0, pitch = 0.0, rollAcc = 0.0, pitchAcc = 0.0;
  updateAngles(ax, ay, az, gx, gy, gz, roll, pitch, rollAcc, pitchAcc);

  // Simple proportional control for roll and pitch stabilization
  float rollCorrection = 0.1 * roll;
  float pitchCorrection = 0.1 * pitch;

  // Map the corrections to adjust motor speeds
  int speedFrontLeft = motorSpeed + rollCorrection - pitchCorrection;
  int speedFrontRight = motorSpeed - rollCorrection - pitchCorrection;
  int speedBackLeft = motorSpeed + rollCorrection + pitchCorrection;
  int speedBackRight = motorSpeed - rollCorrection + pitchCorrection;

  // Constrain the motor speeds to be within the range of 1000-2000 microseconds
  speedFrontLeft = constrain(speedFrontLeft, minMotorSpeed, maxMotorSpeed);
  speedFrontRight = constrain(speedFrontRight, minMotorSpeed, maxMotorSpeed);
  speedBackLeft = constrain(speedBackLeft, minMotorSpeed, maxMotorSpeed);
  speedBackRight = constrain(speedBackRight, minMotorSpeed, maxMotorSpeed);

  // Apply motor speed corrections
  motorFrontLeft.writeMicroseconds(speedFrontLeft);
  motorFrontRight.writeMicroseconds(speedFrontRight);
  motorBackLeft.writeMicroseconds(speedBackLeft);
  motorBackRight.writeMicroseconds(speedBackRight);

  // Check if the drone needs altitude adjustment
  float altitude = bmp.readAltitude(stdAirPressure);
  if (abs(altitude - lastAltitude) > altitudeThreshold) {
    // If the altitude changes significantly, adjust motor speed to stabilize
    if (altitude < initialAltitude + targetAltitude) {
      motorSpeed += 5;  // Slowly increase motor speed
    } else if (altitude > initialAltitude + targetAltitude) {
      motorSpeed -= 5;  // Slowly decrease motor speed
    }
    motorSpeed = constrain(motorSpeed, 1000, maxMotorSpeed);
  }

  lastAltitude = altitude;

  // Add a small delay for smoother control loop
  delay(20);  // Adjust delay as needed for your drone's response time
}

// Graceful landing sequence
void land(float initialAltitude, float motorSpeed, unsigned long landingStartTime) {
  // Check if we've been hovering for 15 seconds
  if (millis() - landingStartTime >= 15000) {
    // Gradually decrease the motor speed over time (5 seconds)
    motorSpeed = map(millis() - landingStartTime, 0, 5000, maxMotorSpeed, minMotorSpeed);
    motorSpeed = constrain(motorSpeed, minMotorSpeed, maxMotorSpeed);

    // Apply motor speed to ESCs
    motorFrontLeft.writeMicroseconds(motorSpeed);
    motorFrontRight.writeMicroseconds(motorSpeed);
    motorBackLeft.writeMicroseconds(motorSpeed);
    motorBackRight.writeMicroseconds(motorSpeed);

    // Stop the motors once the altitude is near zero
    if (bmp.readAltitude(stdAirPressure) < 10) {  // If we are very close to the ground
      motorFrontLeft.writeMicroseconds(minMotorSpeed);
      motorFrontRight.writeMicroseconds(minMotorSpeed);
      motorBackLeft.writeMicroseconds(minMotorSpeed);
      motorBackRight.writeMicroseconds(minMotorSpeed);
      return;  // Motors stop completely
    }
  }
}

void loop() {
  static unsigned long hoverStartTime = 0;
  static unsigned long landingStartTime = 0;
  static float initialAltitude = bmp.readAltitude();
  static float motorSpeed = minMotorSpeed;
  static float lastAltitude = initialAltitude;
  static float targetAltitude = 100.0;  // Target altitude (in centimeters -> appox. 3 ft.) 

  // Takeoff Sequence
  if (bmp.readAltitude(stdAirPressure) <= initialAltitude + targetAltitude) {
    takeoff(initialAltitude, targetAltitude);
    hoverStartTime = millis();
  }

  // Stabilize and hover for 15 seconds
  stabilize(initialAltitude, targetAltitude, motorSpeed, hoverStartTime, lastAltitude);

  // Landing Sequence
  land(initialAltitude, motorSpeed, landingStartTime);
}
