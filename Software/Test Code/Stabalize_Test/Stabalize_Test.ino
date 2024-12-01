#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <IRremote.h>

// IR sensor
IRrecv receiverIR(7);
int IRCode = 0;

// Servo motors (ESCs)
Servo FL, FR, BL, BR;
int speeds[4];

// Motor speed parameters
const int minMotorSpeed = 1000, maxMotorSpeed = 2000, hoverMotorSpeed = 1200;

// MPU6050 instance
MPU6050 mpu;

// Gyro and accelerometer error offsets
typedef struct {
  float X, Y, Z;
} Error;

Error gyroError = {0, 0, 0}, accelError = {0, 0, 0};
float previousErrorPitch = 0.0, integralPitch = 0.0, PitchPID;
float previousErrorRoll = 0.0, integralRoll = 0.0, RollPID;
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
  receiverIR.stopTimer();

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
  /*
  receiverIR.restartTimer();
  Serial.println(F("Plug in the ESCs - wait for confirmation beeps\n"));
  while (IRCode != 2) { checkIRCode(); }
  receiverIR.stopTimer();
  */

  delay(1000);
  previousTime = micros();
}

void loop() {
  receiverIR.restartTimer();
  checkIRCode();
  receiverIR.stopTimer();

  unsigned long currentTime = micros();
  // Sampling rate = 555 Hz
  if (currentTime - previousTime >= 1800 && IRCode != 3) {
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

void checkIRCode(){
  if (receiverIR.decode()) { 
    IRCode = receiverIR.decodedIRData.command; 
    delay(10);
    receiverIR.resume();
  }
  return;
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

  
  int complement; 
  // Apply complementary filter of alpha = 98%
  //pitch = fmod(0.98 * gyroPitch + (0.02) * accPitch + 180.0, 360.0) - 180.0;
  pitch = 0.98 * gyroPitch + (0.02) * accPitch;
  //roll = fmod(0.98 * gyroRoll + (0.02) * accRoll + 180.0, 360.0) - 180.0;
  roll = 0.98 * gyroRoll + (0.02) * accRoll;

  yaw += gyro.ZAxis * elapsedTime;
}

// PID control function
void PIDControl(float elapsedTime) {
  // P = proportional gain -> magnitude of response to error
  // I = integral control -> how fast error is removed (increase ki = decrease response time)
  // D = derivate control -> how far in the future to predict rate of change
  float kp = 1.181, ki = 0.109, kd = 0.0008; // PID constants for pitch
  // error = SP - PV(t) (setpoint AKA 0.0 degreees - process variable at time t AKA current pitch)
  float error = -pitch;
  // u(t) = kp * e(t) - ki * integral_0^t e(x)dx + kd * d/dx * e(t)
  // deviating from base formula due to drift accumuation
  PitchPID = kp * error + ki * (integralPitch += error * elapsedTime) + kd * (error - previousErrorPitch) / elapsedTime;
  previousErrorPitch = error;

  kp = 1.042, ki = 0.103, kd = 0.0005; // PID constants for roll
  error = -roll;
  RollPID = kp * error + ki * (integralRoll + error * elapsedTime) + kd * (error - previousErrorPitch) / elapsedTime;
  previousErrorRoll = error;
}

// Alternative to the abruptness of constrain()
int softClamp(int value, int min, int max) {
  if (value < min) return min + (value - min) / 6;
  if (value > max) return max + (value - max) / 6;
  return value;
}

void adjustMotors() {
  // Scale corrections to keep adjustments moderate
  float pitchScale = 0.3, rollScale = 0.3;

  // Calculate motor speed adjustments w/ soft clamping to enforce bounds (1170 to 1230)
  int targetSpeeds[4] = {
    hoverMotorSpeed - PitchPID * pitchScale - RollPID * rollScale,
    hoverMotorSpeed - PitchPID * pitchScale + RollPID * rollScale,
    hoverMotorSpeed + PitchPID * pitchScale - RollPID * rollScale,
    hoverMotorSpeed + PitchPID * pitchScale + RollPID * rollScale
  };

  // Smooth transition 
  for (int i = 0; i < 4; i++) {
    speeds[i] = softClamp(0.8 * speeds[i] + 0.2 * targetSpeeds[i], 1170, 1230);
  }

  // Write speeds to motors (left side tends to overpower - keep an eye on this)
  applySpeeds(speeds);

  debugOutput();
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
