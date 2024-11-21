#include <wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>

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

// Create an instance of the MPU6050 object
MPU6050 mpu;

// Complementary filter constants
float alpha = 0.98;  // Filter constant (tune as needed)
float dt = 0.01; // Time step

// Initial motor speed (this will be modified later)
int minMotorSpeed = 1000;  // Base motor speed (minimum throttle)
int maxMotorSpeed = 2000;  // Maximum throttle for motors

const float stdAirPressure = 1013.25;

//calibrate and arm All ESC/Motors
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

  delay(2000);
}

void updateAngles(float ax, float ay, float az, float gx, float gy, float gz, float &roll, float &pitch, float &rollAcc, float &pitchAcc) {
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


void stabilize(float motorSpeed, unsigned long hoverStartTime) {
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

  delay(20);
}
void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long hoverStartTime = 0;
  static float motorSpeed = minMotorSpeed;

  //start motors
  motorFrontLeft.writeMicroseconds(minMotorSpeed);
  motorFrontRight.writeMicroseconds(minMotorSpeed);
  motorBackLeft.writeMicroseconds(minMotorSpeed);
  motorBackRight.writeMicroseconds(minMotorSpeed);
  
  //stabalize
  stabalize(motorSpeed, hoverStartTime);
}
