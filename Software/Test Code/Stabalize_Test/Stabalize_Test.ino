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
const float alpha = 0.95; // Complementary filter weight


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

// Might be worth it to explore an offset equation
// as a function of time, if data drift continues to be a problem
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
  findOffset(1000.0);
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
  // Read and adjust gyroscope / accelerometer
  Vector gyro = mpu.readNormalizeGyro();
  gyro.XAxis -= gyroError.X;
  Serial.print(gyro.XAxis); Serial.print(", ");
  gyro.YAxis -= gyroError.Y;
  Serial.print(gyro.YAxis); Serial.print(", ");
  gyro.ZAxis -= gyroError.Z;
  Serial.print(gyro.ZAxis); Serial.print(", ");

  Vector accel = mpu.readNormalizeAccel();
  accel.XAxis -= accelError.X;
  Serial.print(accel.XAxis); Serial.print(", ");
  accel.YAxis -= accelError.Y;
  Serial.print(accel.YAxis); Serial.print(", ");
  accel.ZAxis -= accelError.Z;
  Serial.print(accel.ZAxis); Serial.print(", ");

  // Calculate pitch and roll from gyroscope
  float gyroPitch = pitch + gyro.YAxis * elapsedTime;
  Serial.print(gyroPitch); Serial.print(", ");
  float gyroRoll = roll + gyro.XAxis * elapsedTime; 
  Serial.print(gyroRoll); Serial.print(", ");

  // Calculate pitch and roll from acceleromter
  //float accPitch = (atan2(accel.XAxis - accelError.X, accel.ZAxis - accelError.Z) * 180.0)/PI;
  //float accRoll = (atan2(accel.YAxis - accelError.Y, accel.ZAxis - accelError.Z) * 180.0)/PI;

  float accRoll = (atan(accel.YAxis / sqrt(pow(accel.XAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI) ;
  Serial.print(accRoll); Serial.print(", ");
  float accPitch = (atan(-1.0 * accel.XAxis / sqrt(pow(accel.YAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI); 
  Serial.print(accPitch); Serial.print(", ");

  // Apply complementary filter
  float newPitch = (1 - alpha) * gyroPitch + alpha * accPitch;
  Serial.print(newPitch); Serial.print(", ");
  pitch = (4.0 * pitch + newPitch) / 5.0;
  float newRoll = (1 - alpha) * gyroRoll + alpha * accRoll;
  Serial.print(newRoll); Serial.print(", ");
  roll = (4.0 * roll + newRoll) / 5.0;
  yaw += gyro.ZAxis * elapsedTime;

  // Log values for debugging
  //Serial.print("Pitch: "); 
  Serial.print(pitch); Serial.print(", ");
  //Serial.print("   Roll: "); 
  Serial.print(roll); Serial.print(", ");
}

// PID control function
float PIDControl(float currentAngle, float previousError, float integral, float elapsedTime) {
  float error = setpoint - currentAngle;
  Serial.print(error); Serial.print(", ");
  integral += error * elapsedTime;
  Serial.print(integral); Serial.print(", ");
  float derivative = (error - previousError) / elapsedTime;
  Serial.print(derivative); Serial.print(", ");
  previousError = error;

  return (kp * error + ki * integral + kd * derivative);
}

void adjustMotors(float pitchCorrection, float rollCorrection) {
  // Scale corrections to keep adjustments moderate
  float pitchScale = 0.25; // Adjust scaling factor if needed
  float rollScale = 0.25;

  // Calculate motor speed adjustments
  float newMotorFrontLeftSpeed = hoverMotorSpeed - pitchCorrection * pitchScale + rollCorrection * rollScale;
  float newMotorFrontRightSpeed = hoverMotorSpeed - pitchCorrection * pitchScale - rollCorrection * rollScale;
  float newMotorBackLeftSpeed = hoverMotorSpeed + pitchCorrection * pitchScale + rollCorrection * rollScale;
  float newMotorBackRightSpeed = hoverMotorSpeed + pitchCorrection * pitchScale - rollCorrection * rollScale;

  // Fine adjustment for motor speeds (smoothing and balancing)
  // Bias the old speed by 60% (can be changed based on observations)
  motorFrontLeftSpeed = (3 * motorFrontLeftSpeed + 2 * newMotorFrontLeftSpeed) / 5;  
  motorFrontRightSpeed = (3 * motorFrontRightSpeed + 2 * newMotorFrontRightSpeed) / 5;
  motorBackLeftSpeed = (3 * motorBackLeftSpeed + 2 * newMotorBackLeftSpeed) / 5;
  motorBackRightSpeed = (3 * motorBackRightSpeed + 2 * newMotorBackRightSpeed) / 5;
  
  // Apply limits for more gradual adjustments (1 microsecond is approx. 0.18 degrees)
  float maxSpeedDifference = 40;  // Limit the max difference 80 microseconds
  motorFrontLeftSpeed = constrain(motorFrontLeftSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorFrontRightSpeed = constrain(motorFrontRightSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorBackLeftSpeed = constrain(motorBackLeftSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);
  motorBackRightSpeed = constrain(motorBackRightSpeed, hoverMotorSpeed - maxSpeedDifference, hoverMotorSpeed + maxSpeedDifference);

  // Write speeds to motors (left side tends to overpower - keep an eye on this)
  motorFrontLeft.writeMicroseconds(motorFrontLeftSpeed - 100);
  motorFrontRight.writeMicroseconds(motorFrontRightSpeed);
  motorBackLeft.writeMicroseconds(motorBackLeftSpeed - 100);
  motorBackRight.writeMicroseconds(motorBackRightSpeed);

  // Debugging: Print motor speeds
  //Serial.print("   FL: "); 
  Serial.print(motorFrontLeftSpeed); Serial.print(", ");
  //Serial.print("   FR: "); 
  Serial.print(motorFrontRightSpeed); Serial.print(", ");
  //Serial.print("   BL: "); 
  Serial.print(motorBackLeftSpeed); Serial.print(", ");
  //Serial.print("   BR: "); 
  Serial.println(motorBackRightSpeed);
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
