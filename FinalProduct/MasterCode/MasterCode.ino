#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> 
#include <IRremote.h>
#include "pitches.h"

// Servo motors (ESCs)
Servo FL, FR, BL, BR;
int speeds[4];

// Motor speed parameters
const int minMotorSpeed = 1000, maxMotorSpeed = 2000;
int targetMotorSpeed = 1700;
int IN_FLIGHT = 0;

// Radar pins
#define trigBack 4
#define echoBack 2
#define trigFront 14
#define echoFront 15

// Radars
Servo frontRadar, backRadar;
const float preferredDistance = 60.0; // Preferred distance in cm
const float distanceTolerance = 10.0; // Tolerance of +/- 10cm

// Buzzer
#define BUZZER_PIN 8
const int alarm = NOTE_FS5;

// MPU6050 instace
MPU6050 mpu;

// Gyro and accelerometer error offsets
typedef struct {
  float X, Y, Z;
} Error;

Error gyroError = {0, 0, 0}, accelError = {0, 0, 0};
float previousPitch = 0.0, integralPitch = 0.0, PitchPID;
float previousRoll = 0.0, integralRoll = 0.0, RollPID;
float pitch = 0.0, roll = 0.0, yaw = 0.0;

// BMP280 instance
Adafruit_BMP280 bmp(10);
float altitudeThreshold = 5.0;
const float stdAirPressure = 1013.25;

// IR receiver
IRrecv receiverIR(7);
volatile int IRCode = 0;

// Time tracking
unsigned long previousTime;

// Debugging count
int DEBUG;

// Function prototypes
void applySpeeds(int speeds[]); // Motors
void calibrateSensors(int samples); // MPU6050
void computeAngles(float elapsedTime); // MPU6050
void PIDControl(float elapsedTime); // Motors & MPU6050
void adjustMotors(); // Motors
int softClamp(int value, int min, int max); // Motors
void checkIRCode(); // IR receiver
int calculateDistance(int trigPin, int echoPin); // Radars
void moveForwards(float updatedSpeed);
void moveBackwards(float updatedSpeed);
void follow();
void takeOff();
void land();
void checkObstacle();
void debugOutput(); 

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize BMP280
  bmp.begin();
  Serial.println(F("BMP280 initialized."));

  //setup radars output and input reading
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  digitalWrite(trigFront, LOW);
  Serial.println(F("Front radar initialized."));

  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);
  digitalWrite(trigBack, LOW);
  Serial.println(F("Back radar initialized."));

  // Setup IR sensor
  receiverIR.enableIRIn();
  Serial.println(F("IR Receiver initialized."));

  /// Initialize and calibrate the MPU6050
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
  Serial.println(F("MPU6050 initialized and calibrated."));

  //setup buzzer pin output reading
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println(F("Buzzer initialized."));
  
  for (int i = 0; i < 4; i++) {
    speeds[i] = minMotorSpeed;
  }

  // Attach and arm ESCs
  FL.attach(3); 
  FR.attach(9);
  BL.attach(5);
  BR.attach(6);

  applySpeeds(speeds);
  Serial.println(F("Plug in the ESCs - wait for confirmation beeps"));
  while (IRCode != 2) { checkIRCode(); }
  Serial.println(F("Servo motors successfuly armed -- System Ready!\n"));

  delay(1000);
  previousTime = micros();
}

void loop() {
  checkIRCode();
  unsigned long currentTime = micros();
  float elapsedTime;
  // Sampling rate = 2000 Hz
  if (currentTime - previousTime >= 500) {
    elapsedTime = (currentTime - previousTime) / 1e6; // Convert to seconds
    previousTime = currentTime;
    switch(IRCode) {
      case 1: // Just hover
        Serial.println(F("Hovering..."));
        // Compute angles and apply PID
        computeAngles(elapsedTime);
        PIDControl(elapsedTime);
        // Adjust motors
        adjustMotors();
        //checkObstacle();
        break;
      case 2: // Takeoff sequence
        if (IN_FLIGHT == 0) { takeOff(); }
        Serial.println(F("TAKEOFF SUCCESSFUL!"));
        IRCode = 1;
        break;
      case 3: // Landing sequence
        if (IN_FLIGHT) { land(); Serial.println(F("LANDING SUCCESSFUL!")); }
        break;
      default: // Follow the user
        Serial.println(F("Following..."));
        follow();
        // Compute angles and apply PID
        computeAngles(elapsedTime);
        PIDControl(elapsedTime);
        // Adjust motors
        adjustMotors();
        checkObstacle();
        break;
    }
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
  Serial.print("DEBUG #");
  Serial.print(++DEBUG);
  Serial.print(F(" X: ")); Serial.print(gyro.XAxis); Serial.print(", ");
  Serial.print(F("Y: ")); Serial.print(gyro.YAxis); Serial.print(", ");
  Serial.print(F("Z: ")); Serial.print(gyro.ZAxis); Serial.print(", ");

  Vector accel = mpu.readNormalizeAccel();
  accel.XAxis -= accelError.X, accel.YAxis -= accelError.Y, accel.ZAxis -= accelError.Z;

  // Calculate pitch and roll from gyroscope
  float gyroPitch = pitch + gyro.YAxis * elapsedTime;
  float gyroRoll = roll + gyro.XAxis * elapsedTime; 

  float accRoll = (atan2(accel.YAxis, sqrt(pow(accel.XAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI) ;
  float accPitch = (atan2(-1.0 * accel.XAxis, sqrt(pow(accel.YAxis, 2) + pow(accel.ZAxis, 2))) * 180 / PI); 

  // Apply complementary filter of alpha = 98%
  pitch = (pitch + 0.98 * gyroPitch + (0.02) * accPitch) / 2.0;
  roll = (roll + 0.98 * gyroRoll + (0.02) * accRoll) / 2.0;

  yaw += gyro.ZAxis * elapsedTime;
}

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
  if (value < min) return min + (value - min) / 6;
  if (value > max) return max + (value - max) / 6;
  return value;
}

void adjustMotors() {
  // Scale corrections to keep adjustments moderate
  float scale = 0.2;

  // positive pitch -> back side needs to increase speed
  // positive roll -> right side needs to increase speed
  int targetSpeeds[4] = {
    targetMotorSpeed + (-PitchPID - RollPID) * scale, // Front left
    targetMotorSpeed + (-PitchPID + RollPID) * scale, // Front right
    targetMotorSpeed + (PitchPID - RollPID) * scale, // Back left
    targetMotorSpeed + (PitchPID + RollPID) * scale // Back right
  };

  // Calculate motor speed adjustments w/ soft clamping to smoothly enforce bounds
  for (int i = 0; i < 4; i++) {
    speeds[i] = softClamp(0.8 * speeds[i] + 0.2 * targetSpeeds[i], targetMotorSpeed - 30, targetMotorSpeed + 30);
  }

  // Write speeds to motors 
  applySpeeds(speeds);

  debugOutput();
}

void checkIRCode() {
  if (receiverIR.decode()) { 
    IRCode = receiverIR.decodedIRData.command; 
    delayMicroseconds(10);
    receiverIR.resume();
  }
}

int calculateDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.034/2);
}

void moveForwards(float updatedSpeed) {
  speeds[2] = updatedSpeed, speeds[3] = updatedSpeed;
  applySpeeds(speeds);
}

void moveBackwards(float updatedSpeed) {
  speeds[0] = updatedSpeed, speeds[1] = updatedSpeed;
  applySpeeds(speeds);
}

void follow() {
  backRadar.write(trigBack);
  delayMicroseconds(500);
  float userDistance = calculateDistance(trigBack, echoBack);
  float distanceDifference, motorSpeed;
  
  // Move towards user gradually 
  if (userDistance > (preferredDistance - distanceTolerance)) {
    distanceDifference = userDistance - preferredDistance;
    motorSpeed = map(distanceDifference, 0, preferredDistance, targetMotorSpeed - 50, targetMotorSpeed + 50);
    moveBackwards(motorSpeed);
  }

  // Move away from user gradually
  if (userDistance < (preferredDistance + distanceTolerance)) {
    distanceDifference = preferredDistance - userDistance;
    motorSpeed = map(distanceDifference, 0, preferredDistance, targetMotorSpeed - 50, targetMotorSpeed + 50);  // Gradually increase speed
    moveForwards(motorSpeed);
  }
}

void takeOff() {
  Serial.println(F("Taking off!"));
  float tempTarget = targetMotorSpeed;
  targetMotorSpeed = minMotorSpeed;

  float droneAltitude = bmp.readAltitude(stdAirPressure);
  float targetAltitude = droneAltitude + 1.0; // meters
  int loopNum; // For test/demo purposes
  while (droneAltitude < targetAltitude) {
    droneAltitude = bmp.readAltitude(stdAirPressure);
    float alpha = map(targetAltitude - droneAltitude, -0.5, 1.5, 0, 1);
    unsigned long currentTime = micros();
    float elapsedTime;
    // Sampling rate = 2000 Hz
    if (currentTime - previousTime >= 500) {
      elapsedTime = (currentTime - previousTime) / 1e6; // Convert to seconds
      previousTime = currentTime;
      
      // Slowly increase speed
      targetMotorSpeed = alpha * targetMotorSpeed + (1 - alpha) * tempTarget;

      // Compute angles and apply PID
      computeAngles(elapsedTime);
      PIDControl(elapsedTime);

      // Adjust motors
      adjustMotors();  
      delayMicroseconds(100);  // Adjust delay for smoother takeoff
    }
  }
  targetMotorSpeed = tempTarget;
  IN_FLIGHT = 1;
}

void land() {
  int tempTarget = targetMotorSpeed;
  Serial.print(F("Landing..."));
  while (targetMotorSpeed > minMotorSpeed) {
    unsigned long currentTime = micros();
    // Sampling rate = 2000 Hz
    if (currentTime - previousTime >= 500) {
      float elapsedTime = (currentTime - previousTime) / 1e6; // Convert to seconds
      previousTime = currentTime;

      // Compute angles and apply PID
      computeAngles(elapsedTime);
      PIDControl(elapsedTime);

      // Adjust motors
      adjustMotors(); 
      targetMotorSpeed -= 3;
      delayMicroseconds(100);
    }
  }
  int off[4] = {0,0,0,0};
  applySpeeds(off);
  IN_FLIGHT = 0;
  targetMotorSpeed = tempTarget;
}

void checkObstacle() {
  frontRadar.write(trigFront);
  delayMicroseconds(500);
  int obstacleDist = calculateDistance(trigFront, echoFront);
  if (obstacleDist < 11.11) {
    receiverIR.stopTimer();
    tone(BUZZER_PIN, alarm);
    delay(250);
    noTone(BUZZER_PIN);
    receiverIR.restartTimer();
    delayMicroseconds(100);
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
  
  Serial.print(F("   FL: ")); 
  Serial.print(speeds[0]); Serial.print(F(","));
  Serial.print(F("   FR: ")); 
  Serial.print(speeds[1]); Serial.print(F(","));
  Serial.print(F("   BL: ")); 
  Serial.print(speeds[2]); Serial.print(F(","));
  Serial.print(F("   BR: "));
  Serial.println(speeds[3]);
}
