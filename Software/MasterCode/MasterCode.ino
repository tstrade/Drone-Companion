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

// Define variables for sensor readings
float ax, ay, az;  // Accelerometer data
float gx, gy, gz;  // Gyroscope data

// Define PID constants for stabilization (values are placeholders)
float Kp = 1.0;  // Proportional gain for pitch and roll control
float Ki = 0.0;  // Integral gain (optional)
float Kd = 0.5;  // Derivative gain for pitch and roll control

// Initial motor speed (this will be modified later)
int motorSpeed = 1000;  // Base motor speed (minimum throttle)

// Angle offsets for the MPU6050 (these might need to be calibrated)
float offsetRoll = 0.0;
float offsetPitch = 0.0;
float offsetYaw = 0.0;

void setESCSpeed(int speed) {
    motorBackRight.write(speed);
    motorFrontLeft.write(speed);
    motorBackLeft.write(speed);
    motorFrontRight.write(speed);
}

void armESC() {
    Serial.print("PLUG IN NOW!\n");
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
  // Initialize serial communication for debugging
  Serial.begin(115200);
  
  // Initialize the MPU6050
  Wire.begin();

  // Check if the MPU6050 is connected
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("MPU6050 connection failed");
    delay(500);
  }

  mpu.calibrateGyro();
  // Attach ESCs to the appropriate pins
  motorFrontLeft.attach(3);  // Pin 3 for front left motor
  motorFrontRight.attach(5); // Pin 5 for front right motor
  motorBackLeft.attach(6);   // Pin 6 for back left motor
  motorBackRight.attach(9);  // Pin 9 for back right motor
  armESC();
  /* Set initial motor speeds (idle speed for ESCs)
  motorFrontLeft.writeMicroseconds(motorSpeed);
  motorFrontRight.writeMicroseconds(motorSpeed);
  motorBackLeft.writeMicroseconds(motorSpeed);
  motorBackRight.writeMicroseconds(motorSpeed);
  */
}

void loop() {
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
  speedFrontLeft = constrain(speedFrontLeft, 1000, 2000);
  speedFrontRight = constrain(speedFrontRight, 1000, 2000);
  speedBackLeft = constrain(speedBackLeft, 1000, 2000);
  speedBackRight = constrain(speedBackRight, 1000, 2000);

  // Set the motor speeds
  motorFrontLeft.writeMicroseconds(speedFrontLeft);
  motorFrontRight.writeMicroseconds(speedFrontRight);
  motorBackLeft.writeMicroseconds(speedBackLeft);
  motorBackRight.writeMicroseconds(speedBackRight);

  // Print debug information to the serial monitor
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Front Left Speed: ");
  Serial.print(speedFrontLeft);
  Serial.print(" | Front Right Speed: ");
  Serial.print(speedFrontRight);
  Serial.print(" | Back Left Speed: ");
  Serial.print(speedBackLeft);
  Serial.print(" | Back Right Speed: ");
  Serial.println(speedBackRight);

  // Add a small delay for smoother control loop
  delay(20);  // Adjust delay as needed for your drone's response time
}
