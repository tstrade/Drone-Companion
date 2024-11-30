#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// Define the servo motors (ESCs)
Servo motorBackRight;
Servo motorFrontLeft;
Servo motorBackLeft;
Servo motorFrontRight;

// Create an instance of the MPU6050 object
MPU6050 mpu;

// Gyro calibration offsets
typedef struct Error {
  float X;
  float Y;
  float Z;
} Error;

Error gyroError;
Error accelError;

float pitch = 0, roll = 0, yaw = 0;

// PID variables
const float setpoint = 0.0;  // Desired pitch/roll angle (0 degrees)
float previousErrorPitch = 0, integralPitch = 0;
float previousErrorRoll = 0, integralRoll = 0;
const float kp = 1.0, ki = 0.05, kd = 0.5;  // PID constants
const float alpha = 0.98; // Complementary filter weight


// Time tracking
unsigned long previousTime;
const unsigned long sampleRate = 1600; // 625 Hz

// Define relevant motor speeds
const int minMotorSpeed = 1000;  // Minimum throttle
const int maxMotorSpeed = 2000;  // Maximum throttle
const int hoverMotorSpeed = 1200;  // Base throttle

// Initial motor speeds for smoothing
float motorFrontLeftSpeed = hoverMotorSpeed;
float motorFrontRightSpeed = hoverMotorSpeed;
float motorBackLeftSpeed = hoverMotorSpeed;
float motorBackRightSpeed = hoverMotorSpeed;

void findOffset(float samples) {
  int i;

  Vector gyro;
  gyroError.X = 0.0;
  gyroError.Y = 0.0;
  gyroError.Z = 0.0;

  Vector accel;
  accelError.X = 0.0;
  accelError.Y = 0.0;
  accelError.Z = 0.0;

  for (i = 0; i < samples; i++) {
    gyro = mpu.readNormalizeGyro();
    gyroError.X += gyro.XAxis;
    gyroError.Y += gyro.YAxis;
    gyroError.Z += gyro.ZAxis;

    accel = mpu.readNormalizeAccel();
    accelError.X += accel.XAxis;
    accelError.Y += accel.YAxis;
    accelError.Z += accel.ZAxis;
    delay(1);
  }
  gyroError.X /= samples;
  gyroError.Y /= samples;
  gyroError.Z /= samples;

  accelError.X /= samples;
  accelError.Y /= samples;
  accelError.Z /= samples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize and calibrate the MPU6050
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
    Serial.println("Failed to find MPU6050 chip, checking wiring!");
    delay(500);
  }

  mpu.calibrateGyro(1000);
  mpu.setThreshold(3);
  findOffset(1000);
  Serial.println("MPU6050 initialized and calibrated.");

  // Attach ESCs
  motorFrontLeft.attach(3); 
  motorFrontRight.attach(9);
  motorBackLeft.attach(5); 
  motorBackRight.attach(6);

  motorFrontLeft.write(minMotorSpeed);
  motorFrontRight.write(minMotorSpeed);
  motorBackLeft.write(minMotorSpeed);
  motorBackRight.write(minMotorSpeed);

  Serial.println("Plug in the ESCs - wait for confirmation beeps\n");
  while (!Serial.available()) {}
  Serial.read();

  delay(1000);
  previousTime = micros();
}

void computeAngles(float elapsedTime) {
  // Read gyroscope and accelerometer
  Vector gyro = mpu.readNormalizeGyro();
  Vector accel = mpu.readNormalizeAccel();

  // Calculate pitch and roll from gyroscope
  float gyroPitch = pitch + gyro.YAxis * elapsedTime - gyroError.Y;
  float gyroRoll = roll + gyro.XAxis * elapsedTime - gyroError.X; 

  // Calculate pitch and roll from acceleromter
  float accPitch = (atan2(accel.XAxis - accelError.X, accel.ZAxis - accelError.Z) * 180)/PI;
  float accRoll = (atan2(accel.YAxis - accelError.Y, accel.ZAxis - accelError.Z) * 180.0)/PI;
  
  // Apply complementary filter
  pitch = (1 - alpha) * gyroPitch + alpha * accPitch;
  roll = (1 - alpha) * gyroRoll + alpha * accRoll;
  yaw += gyro.ZAxis * elapsedTime - gyroError.Z;

}

// PID control function
float PIDControl(float currentAngle, float previousError, float integral, float elapsedTime) {
  float error = setpoint - currentAngle;
  integral += error * elapsedTime;
  float derivative = (error - previousError) / elapsedTime;
  previousError = error;

  return (kp * error + ki * integral + kd * derivative);
}

void adjustMotors(float pitchCorrection, float rollCorrection) {
  // Scale corrections to keep adjustments moderate
  float pitchScale = 0.2; // Adjust scaling factor if needed
  float rollScale = 0.3;

  // Calculate motor speed adjustments
  float newMotorFrontLeftSpeed = hoverMotorSpeed - pitchCorrection * pitchScale - rollCorrection * rollScale;
  float newMotorFrontRightSpeed = hoverMotorSpeed + pitchCorrection * pitchScale - rollCorrection * rollScale;
  float newMotorBackLeftSpeed = hoverMotorSpeed - pitchCorrection * pitchScale + rollCorrection * rollScale;
  float newMotorBackRightSpeed = hoverMotorSpeed + pitchCorrection * pitchScale + rollCorrection * rollScale;

   // Log values for debugging
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("   Roll: "); Serial.print(roll);

  // Fine adjustment for motor speeds (smoothing and balancing)
  //       i.e., motor speeds can only change by increments of 20% of the 
  //             difference between the new and current speeds (can be tuned)
  motorFrontLeftSpeed = (3 * motorFrontLeftSpeed + 2 * newMotorFrontLeftSpeed) / 5;  
  motorFrontRightSpeed = (3 * motorFrontRightSpeed + 2 * newMotorFrontRightSpeed) / 5;
  motorBackLeftSpeed = (3 * motorBackLeftSpeed + 2 * newMotorBackLeftSpeed) / 5;
  motorBackRightSpeed = (3 * motorBackRightSpeed + 2 * newMotorBackRightSpeed) / 5;
  
  // Apply limits for more gradual adjustments (1 microsecond is approx. 0.18 degrees)
  float maxSpeedDifference = 30;  // Limit the max difference 60 microseconds
  motorFrontLeftSpeed = constrain(motorFrontLeftSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorFrontRightSpeed = constrain(motorFrontRightSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorBackLeftSpeed = constrain(motorBackLeftSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorBackRightSpeed = constrain(motorBackRightSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);

  // Write speeds to motors
  motorFrontLeft.writeMicroseconds(motorFrontLeftSpeed);
  motorFrontRight.writeMicroseconds(motorFrontRightSpeed);
  motorBackLeft.writeMicroseconds(motorBackLeftSpeed);
  motorBackRight.writeMicroseconds(motorBackRightSpeed);

  // Debugging: Print motor speeds
  Serial.print("   FL: "); Serial.print(motorFrontLeftSpeed);
  Serial.print("   FR: "); Serial.print(motorFrontRightSpeed);
  Serial.print("   BL: "); Serial.print(motorBackLeftSpeed);
  Serial.print("   BR: "); Serial.println(motorBackRightSpeed);
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - previousTime >= sampleRate) {
    float elapsedTime = (currentTime - previousTime) / 1000000.0; // Convert to seconds

    // Read MPU6050 and compute angles
    computeAngles(elapsedTime);

    // PID Controller for pitch and roll stabilization
    float pitchCorrection = PIDControl(pitch, previousErrorPitch, integralPitch, elapsedTime);
    float rollCorrection = PIDControl(roll, previousErrorRoll, integralRoll, elapsedTime);

    // Adjust motors
    adjustMotors(pitchCorrection, rollCorrection);
  }
}
