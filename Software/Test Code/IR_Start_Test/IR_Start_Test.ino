#include <Servo.h>
#include <IRremote.h>

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

//Initialize IR reciever in an IRrecv object
IRrecv recieverIR(7);

//the code read from IR remote
int IRCode = 0; //2 is On, 3 is Off

int minMotorSpeed = 1000;  // Minimum throttle
int maxMotorSpeed = 2000;  // Maximum throttle

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

  //setup IR
  recieverIR.enableIRIn();
}

int checkIRCode(int &IRCode){
  if(recieverIR.decode()) {
    Serial.println(recieverIR.decodedIRData.command);
    IRCode = recieverIR.decodedIRData.command; //this will not return the integer. I do not have the USB to check how to recieve the integer from the given code.
    //P.S. I know where the USB is, it is on my desk. i will have it tomorrow, November 21st.
    recieverIR.resume();
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  if(IRCode == 2){
    //reset IRCode
    IRCode = 0;
    //loop so it doesnt leave this if statement
    while(true){
      //start motors
      motorBackRight.write(minMotorSpeed);
      motorFrontLeft.write(minMotorSpeed);
      motorBackLeft.write(minMotorSpeed);
      motorFrontRight.write(minMotorSpeed);
      if(IRCode == 2){
        //reset IRCode
        IRCode = 0;
        //detach motors
        motorBackRight.detach();
        motorFrontLeft.detach();
        motorBackLeft.detach();
        motorFrontRight.detach();
      }
    }
  }
}
