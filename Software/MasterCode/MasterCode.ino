#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> 

#define BACKRIGHT 9
#define FRONTLEFT 3
#define BACKLEFT 6
#define FRONTRIGHT 5

// Define the servo motors (ESCs)
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

// Create an instance of the MPU6050 object
MPU6050 mpu;
// Create an instance of the BMP280 object
Adafruit_BMP280 bmp(10);

// Define variables for sensor readings
float ax, ay, az;  // Accelerometer data
float gx, gy, gz;  // Gyroscope data
float altitude;  // Altitude reading from BMP280
float targetAltitude = 100.0;  // Target hover altitude in centimeters

// Define PID constants for stabilization (values are placeholders)
float Kp = 1.0;  // Proportional gain for pitch and roll control
float Ki = 0.0;  // Integral gain (optional)
float Kd = 0.5;  // Derivative gain for pitch and roll control

// Initial motor speed (this will be modified later)
int motorSpeed = 1000;  // Base motor speed (minimum throttle)
int maxMotorSpeed = 2000;  // Maximum throttle for motors

// Angle offsets for the MPU6050 (these might need to be calibrated)
float offsetRoll = 0.0;
float offsetPitch = 0.0;
float offsetYaw = 0.0;

// Altitude control variables
float initialAltitude;  // Altitude at the start (reference altitude)
float lastAltitude = 0.0;
float altitudeSpeed = 0.0;
float altitudeThreshold = 5.0;
const float stdAirPressure = 1013.25;
unsigned long hoverStartTime = 0;  // To track the time of hover
unsigned long landingStartTime = 0;  // To track the landing start time

void armESC() {
    // Set high throttle
    delay(2000);
    motorBackRight.write(2000);
    motorFrontLeft.write(2000);
    motorBackLeft.write(2000);
    motorFrontRight.write(2000);
    delay(5000);
    // Set low throttle
    motorBackRight.write(1000);
    motorFrontLeft.write(1000);
    motorBackLeft.write(1000);
    motorFrontRight.write(1000);
    delay(3000);   
    // Arm ESCs
    motorBackRight.write(500);
    motorFrontLeft.write(500);
    motorBackLeft.write(500);
    motorFrontRight.write(500);
}

void setup() {
  // Initialize and calibrate the MPU6050
  Wire.begin();
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    delay(500);
  }
  mpu.calibrateGyro();

  // Initialize BMP280
  bmp.begin();
  // Attach ESCs to the appropriate pins
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(5); // Pin 5 for front right motor
  motorBackLeft.attach(6);   // Pin 6 for back left motor
  motorBackRight.attach(9);  // Pin 9 for back right motor
  armESC();

  // Set initial motor speeds (idle speed for ESCs)
  motorFrontLeft.writeMicroseconds(motorSpeed);
  motorFrontRight.writeMicroseconds(motorSpeed);
  motorBackLeft.writeMicroseconds(motorSpeed);
  motorBackRight.writeMicroseconds(motorSpeed);
  //
  delay(2000);
}

// Takeoff sequence: gradually increase motor speeds
void takeoff() {
  motorSpeed = 1000;

  while (altitude < targetAltitude) {
    altitude = bmp.readAltitude(stdAirPressure);  // Read current altitude

    motorSpeed = map(altitude, 0, targetAltitude, 1000, maxMotorSpeed);  // Gradually increase speed
    motorSpeed = constrain(motorSpeed, 1000, maxMotorSpeed);  // Ensure speed is within bounds

    // Apply motor speed to ESCs
    motorFrontLeft.writeMicroseconds(motorSpeed);
    motorFrontRight.writeMicroseconds(motorSpeed);
    motorBackLeft.writeMicroseconds(motorSpeed);
    motorBackRight.writeMicroseconds(motorSpeed);

    delay(50);  // Adjust delay for smoother takeoff
  }
  hoverStartTime = millis();  // Start the hover timer
}

// Hover and stabilize the drone
void stabilize() {
  // Read accelerometer and gyroscope data
  Vector ACCEL = mpu.readNormalizeAccel();
  ax = ACCEL.XAxis;
  ay = ACCEL.YAxis;
  az = ACCEL.ZAxis;
  Vector GYRO = mpu.readNormalizeGyro();
  gx = GYRO.XAxis;
  gy = GYRO.YAxis;
  gz = GYRO.ZAxis;

  // Convert accelerometer values to angles (simple approximation for stabilization)
  float roll = atan2(ay, az) * 180.0 / PI - offsetRoll;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - offsetPitch;

  // Simple proportional control for roll and pitch stabilization
  float rollCorrection = Kp * roll;
  float pitchCorrection = Kp * pitch;

  // Map the corrections to adjust motor speeds
  int speedFrontLeft = motorSpeed + rollCorrection - pitchCorrection;
  int speedFrontRight = motorSpeed - rollCorrection - pitchCorrection;
  int speedBackLeft = motorSpeed + rollCorrection + pitchCorrection;
  int speedBackRight = motorSpeed - rollCorrection + pitchCorrection;

  // Constrain the motor speeds to be within the range of 1000-2000 microseconds
  speedFrontLeft = constrain(speedFrontLeft, 1000, maxMotorSpeed);
  speedFrontRight = constrain(speedFrontRight, 1000, maxMotorSpeed);
  speedBackLeft = constrain(speedBackLeft, 1000, maxMotorSpeed);
  speedBackRight = constrain(speedBackRight, 1000, maxMotorSpeed);

  // Apply motor speed corrections
  motorFrontLeft.writeMicroseconds(speedFrontLeft);
  motorFrontRight.writeMicroseconds(speedFrontRight);
  motorBackLeft.writeMicroseconds(speedBackLeft);
  motorBackRight.writeMicroseconds(speedBackRight);

  // Check if the drone needs altitude adjustment
  altitude = bmp.readAltitude(stdAirPressure);
  if (abs(altitude - lastAltitude) > altitudeThreshold) {
    // If the altitude changes significantly, adjust motor speed to stabilize
    if (altitude < targetAltitude) {
      motorSpeed += 5;  // Slowly increase motor speed
    } else if (altitude > targetAltitude) {
      motorSpeed -= 5;  // Slowly decrease motor speed
    }
    motorSpeed = constrain(motorSpeed, 1000, maxMotorSpeed);
  }

  lastAltitude = altitude;

  // Add a small delay for smoother control loop
  delay(20);  // Adjust delay as needed for your drone's response time
}

// Graceful landing sequence
void land() {
  // Check if we've been hovering for 15 seconds
  if (millis() - hoverStartTime >= 15000) {
    if (landingStartTime == 0) {
      landingStartTime = millis();  // Start the landing process
    }
    
    // Gradually decrease the motor speed over time
    motorSpeed = map(millis() - landingStartTime, 0, 5000, maxMotorSpeed, 1000);  // Gradual descent (5 seconds)
    motorSpeed = constrain(motorSpeed, 1000, maxMotorSpeed);

    // Apply motor speed to ESCs
    motorFrontLeft.writeMicroseconds(motorSpeed);
    motorFrontRight.writeMicroseconds(motorSpeed);
    motorBackLeft.writeMicroseconds(motorSpeed);
    motorBackRight.writeMicroseconds(motorSpeed);

    // Stop the motors once the altitude is near zero
    if (altitude < 10) {  // If we are very close to the ground
      motorFrontLeft.writeMicroseconds(1000);
      motorFrontRight.writeMicroseconds(1000);
      motorBackLeft.writeMicroseconds(1000);
      motorBackRight.writeMicroseconds(1000);
      return;  // Motors stop completely
    }
  }
}

void loop() {
  // Run the takeoff sequence
  takeoff();

  // Once takeoff is complete, stabilize and hover
  while (true) {
    stabilize();
    land();
  }
}
