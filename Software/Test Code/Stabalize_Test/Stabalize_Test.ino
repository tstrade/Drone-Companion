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
struct gyroError {
  float X;
  float Y;
  float Z;
} gyroError;

float pitch = 0, roll = 0, yaw = 0;

// PID variables
const float setpoint = 0.0;  // Desired pitch/roll angle (0 degrees)
float previousErrorPitch = 0, integralPitch = 0;
float previousErrorRoll = 0, integralRoll = 0;
const float kp = 2.0, ki = 0.05, kd = 1.0;  // PID constants
const float alpha = 0.96; // Complementary filter weight


// Time tracking
unsigned long previousTime;

// Initial motor speed 
const int minMotorSpeed = 1000;  // Minimum throttle
const int maxMotorSpeed = 2000;  // Maximum throttle
const int hoverMotorSpeed = 1150;  // Base throttle

void setup() {
  Serial.begin(115200);
  // Initialize and calibrate the MPU6050
  Wire.begin();
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Failed to find MPU6050 chip, checking wiring!");
    delay(500);
  }
  //calibrateGyroscope();
  mpu.calibrateGyro(500);

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

  delay(3000);
  previousTime = micros();
}

void computeAngles() {
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - previousTime) / 1000000.0;  // Convert to seconds
  previousTime = currentTime;
  
  // Read gyroscope and accelerometer
  Vector gyro = mpu.readNormalizeGyro();
  Vector accel = mpu.readNormalizeAccel();

  // Calculate pitch and roll from gyroscope
  float gyroPitch = pitch + gyro.XAxis * elapsedTime;
  float gyroRoll = roll + gyro.YAxis * elapsedTime; 

  // Calculate pitch and roll from acceleromter
  float accPitch = -(atan2(accel.XAxis, sqrt(accel.YAxis * accel.YAxis + accel.ZAxis * accel.ZAxis)) * 180)/PI;
  float accRoll = (atan2(accel.YAxis, accel.ZAxis) * 180)/PI;

  

  // Apply complementary filter
  pitch = alpha * gyroPitch + (1 - alpha) * accPitch;
  roll = alpha * gyroRoll + (1 - alpha) * accRoll;
  yaw += gyro.ZAxis * elapsedTime;

}

// PID control function
float PIDControl(float currentAngle, float previousError, float integral) {
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - previousTime) / 1000000.0;  // Convert ms to s
  previousTime = currentTime;

  float error = setpoint - currentAngle;
  integral += error * elapsedTime;
  float derivative = (elapsedTime > 0) ? (error - previousError) / elapsedTime : 0;
  previousError = error;

  return (kp * error + ki * integral + kd * derivative);
}

void adjustMotors(float pitchCorrection, float rollCorrection) {
  // Calculate motor speeds
  int motorFrontLeftSpeed = map(hoverMotorSpeed - pitchCorrection - rollCorrection, 
                                -500, 500, minMotorSpeed, maxMotorSpeed);
  int motorFrontRightSpeed = map(hoverMotorSpeed - pitchCorrection + rollCorrection, 
                                 -500, 500, minMotorSpeed, maxMotorSpeed);
  int motorBackLeftSpeed = map(hoverMotorSpeed + pitchCorrection - rollCorrection, 
                               -500, 500, minMotorSpeed, maxMotorSpeed);
  int motorBackRightSpeed = map(hoverMotorSpeed + pitchCorrection + rollCorrection, 
                                -500, 500, minMotorSpeed, maxMotorSpeed);

  motorFrontLeft.writeMicroseconds(motorFrontLeftSpeed);
  motorFrontRight.writeMicroseconds(motorFrontRightSpeed);
  motorBackLeft.writeMicroseconds(motorBackLeftSpeed);
  motorBackRight.writeMicroseconds(motorBackRightSpeed);
}

void loop() {
  // Read MPU6050 and compute angles
  computeAngles();

  // PID Controller for pitch and roll stabilization
  float pitchCorrection = PIDControl(pitch, previousErrorPitch, integralPitch);
  float rollCorrection = PIDControl(roll, previousErrorRoll, integralRoll);

  // Adjust motors
  adjustMotors(pitchCorrection, rollCorrection);
h
  delay(10);
}
