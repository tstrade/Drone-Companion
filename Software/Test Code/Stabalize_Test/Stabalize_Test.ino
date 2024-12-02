#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <IRremote.h>
#include "PinChangeInterrupt.h"

// IR sensor
IRrecv receiverIR(7);
volatile int IRCode = 0;

// Servo motors (ESCs)
Servo FL, FR, BL, BR;
int speeds[4];

// Motor speed parameters
const int minMotorSpeed = 1000, maxMotorSpeed = 2000;
int hoverMotorSpeed = 1850;

// MPU6050 instance
MPU6050 mpu;

// Gyro and accelerometer error offsets
typedef struct {
  float X, Y, Z;
} Error;

Error gyroError = {0, 0, 0}, accelError = {0, 0, 0};
float previousPitch = 0.0, integralPitch = 0.0, PitchPID;
float previousRoll = 0.0, integralRoll = 0.0, RollPID;
float pitch = 0.0, roll = 0.0, yaw = 0.0;

// Time tracking
unsigned long previousTime;

// Function prototypes
void applySpeeds(int speeds[]);
void calibrateSensors(int samples);
void findPRYError(int samples);
void computeAngles(float elapsedTime);
void PIDControl(float elapsedTime);
void adjustMotors();
int softClamp(int value, int min, int max);
void checkIRCode();
void debugOutput();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Setup IR sensor
  receiverIR.enableIRIn();

  // Initialize and calibrate the MPU6050
  Serial.println(F("Initializing MPU6050"));
  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
    Serial.println(F("Failed to find MPU6050 chip, checking wiring!"));
    delay(500);
  }

  // Built-in calibration (not great)
  mpu.calibrateGyro(1000);
  mpu.setThreshold(3);
  
  // Manual calibration to find offsets
  calibrateSensors(500);
  //findPRYError(500);
  Serial.println(F("MPU6050 initialized and calibrated."));

  for (int i = 0; i < 4; i++) {
    speeds[i] = minMotorSpeed;
  }

  // Attach ESCs
  FL.attach(3); 
  FR.attach(9);
  BL.attach(5);
  BR.attach(6);

  applySpeeds(speeds);
  
  Serial.println(F("Plug in the ESCs - wait for confirmation beeps\n"));
  while (IRCode != 2) { checkIRCode(); }

  delay(1000);
  previousTime = micros();
}

void loop() {
  checkIRCode();
  unsigned long currentTime = micros();
  if (IRCode == 8) hoverMotorSpeed = 1700;
  // Sampling rate = 2000 Hz
  if (currentTime - previousTime >= 500 && IRCode != 3) {
    float elapsedTime = (currentTime - previousTime) / 1e6; // Convert to seconds
    previousTime = currentTime;
    
    // Compute angles and apply PID
    computeAngles(elapsedTime);
    PIDControl(elapsedTime);

    // Adjust motors
    adjustMotors();
    
  } else if (IRCode == 3) {
    FL.writeMicroseconds(0);
    FR.writeMicroseconds(0);
    BL.writeMicroseconds(0);
    BR.writeMicroseconds(0);
  }
}

void applySpeeds(int speeds[]) {
  FL.writeMicroseconds(speeds[0]);
  FR.writeMicroseconds(speeds[1]);
  BL.writeMicroseconds(speeds[2]);
  BR.writeMicroseconds(speeds[3]);
}

void calibrateSensors(int samples) {
  Vector gyro, accel;

  for (int i = 0; i < samples; i++) {
    gyro = mpu.readNormalizeGyro();
    accel = mpu.readNormalizeAccel();
    
    gyroError.X += gyro.XAxis;
    gyroError.Y += gyro.YAxis;
    gyroError.Z += gyro.ZAxis;
    
    accelError.X += accel.XAxis;
    accelError.Y += accel.YAxis;
    accelError.Z += accel.ZAxis;  

    delay(1);
  }
  gyroError.X /= samples, gyroError.Y /= samples, gyroError.Z /= samples;
  accelError.X /= samples, accelError.Y /= samples, accelError.Z /= samples;

  Serial.println(F("Calibration complete. Verifying error..."));
}

void computeAngles(float elapsedTime) {
  // Read and adjust gyroscope / accelerometer
  Vector gyro = mpu.readNormalizeGyro();
  gyro.XAxis -= gyroError.X, gyro.YAxis -= gyroError.Y, gyro.ZAxis -= gyroError.Z;
  Serial.print(F("X: ")); Serial.print(gyro.XAxis); Serial.print(", ");
  Serial.print(F("Y: ")); Serial.print(gyro.YAxis); Serial.print(", ");
  Serial.print(F("Z: ")); Serial.print(gyro.ZAxis); Serial.print(", ");

  Vector accel = mpu.readNormalizeAccel();
  accel.XAxis -= accelError.X, accel.YAxis -= accelError.Y, accel.ZAxis -= accelError.Z;

  // Calculate pitch and roll from gyroscope
  float gyroPitch = pitch + gyro.YAxis * elapsedTime;
  float gyroRoll = roll + gyro.XAxis * elapsedTime; 

  float accRoll = (atan2(accel.YAxis, sqrt(pow(accel.XAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI) ;
  float accPitch = (atan2(-1.0 * accel.XAxis, sqrt(pow(accel.YAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI); 

  // Apply complementary filter of alpha = 95%
  pitch = (pitch + 0.98 * gyroPitch + (0.02) * accPitch) / 2.0;
  roll = (roll + 0.98 * gyroRoll + (0.02) * accRoll) / 2.0;

  yaw += gyro.ZAxis * elapsedTime;
}

// PID control function
void PIDControl(float elapsedTime) {
  // P = proportional gain -> magnitude of response to error
  // I = integral control -> how fast error is removed (increase ki = decrease response time)
  // D = derivate control -> how far in the future to predict rate of change
  float kp = 1.01, ki = 0.6, kd = 0.00003; // PID constants

  // u(t) = kp * e(t) + ki * integral_0^t e(x)dx + kd * d/dx * e(t)
  PitchPID = kp * pitch + ki * (integralPitch += pitch * elapsedTime) + kd * (pitch - previousPitch) / elapsedTime;
  previousPitch = pitch;
  RollPID = kp * roll + ki * (integralRoll += pitch * elapsedTime) + kd * (pitch - previousPitch) / elapsedTime;
  previousRoll = roll;
}

// Alternative to the abruptness of constrain()
int softClamp(int value, int min, int max) {
  if (value < min) return min + (value - min) / 5;
  if (value > max) return max + (value - max) / 5;
  return value;
}

void adjustMotors() {
  // Scale corrections to keep adjustments moderate
  float scale = 0.2;

  // positive pitch -> back side needs to increase speed
  // positive roll -> right side needs to increase speed
  int targetSpeeds[4] = {
    hoverMotorSpeed + (-PitchPID - RollPID) * scale - 20, // Front left
    hoverMotorSpeed + (-PitchPID + RollPID) * scale - 5, // Front right
    hoverMotorSpeed + (PitchPID - RollPID) * scale + 20, // Back left
    hoverMotorSpeed + (PitchPID + RollPID) * scale + 10 // Back right
  };

  // Calculate motor speed adjustments w/ soft clamping to smoothly enforce bounds
  for (int i = 0; i < 4; i++) {
    speeds[i] = softClamp(0.8 * speeds[i] + 0.2 * targetSpeeds[i], hoverMotorSpeed - 30, hoverMotorSpeed + 30);
  }

  // Write speeds to motors (left side tends to overpower - keep an eye on this)
  applySpeeds(speeds);

  debugOutput();
}

void checkIRCode() {
  if (receiverIR.decode()) { 
    IRCode = receiverIR.decodedIRData.command; 
    delay(10);
    receiverIR.resume();
  }
}

void debugOutput() {
  // Log values for debugging
  Serial.print(F("Pitch: ")); 
  Serial.print(pitch); Serial.print(F(","));
  Serial.print(F("   Roll: ")); 
  Serial.print(roll); Serial.print(F(","));
  Serial.print(F("   Yaw: ")); 
  Serial.print(yaw); Serial.print(F(","));
  
  // Debugging: Print motor speeds
  Serial.print(F("   FL: ")); 
  Serial.print(speeds[0]); Serial.print(F(","));
  Serial.print(F("   FR: ")); 
  Serial.print(speeds[1]); Serial.print(F(","));
  Serial.print(F("   BL: ")); 
  Serial.print(speeds[2]); Serial.print(F(","));
  Serial.print(F("   BR: "));
  Serial.println(speeds[3]);
}
