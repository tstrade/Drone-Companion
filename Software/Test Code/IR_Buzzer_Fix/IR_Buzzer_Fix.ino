#include <Servo.h>
#include <IRremote.h>
#include "pitches.h"

// Define buzzer
#define BUZZER_PIN 8
const int alert_tone = NOTE_FS5;

// Define radars
#define trigFront 14
#define echoFront 15
#define trigBack 4
#define echoBack 2
Servo frontRadar, backRadar;

// Define IR sensor
IRrecv receiverIR(7);
int IRCode  = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup IR sensor
  receiverIR.enableIRIn();

  // Setup radars
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);

  // Setup buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

void checkIRCode(){
  if (receiverIR.decode()) { 
    IRCode = receiverIR.decodedIRData.command; 
    delay(10);
    receiverIR.resume();
  }
  return;
}

float calculateDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return (pulseIn(echo, HIGH) * 0.034/2);
}

void loop() {
  // Receive input from remote
  checkIRCode();
  Serial.println(IRCode);

  // Check user distance to adjust motors
  backRadar.write(trigBack);
  delay(500);
  int userDist = calculateDistance(trigBack, echoBack);
  Serial.print("User distance (cm) is: ");
  Serial.println(userDist);
  
  // Detect obstacle distance to alert buzzer
  frontRadar.write(trigFront);
  delay(500);
  int obstacleDist = calculateDistance(trigFront, echoFront);
  Serial.print("Obstacle distance (cm) is: ");
  Serial.println(obstacleDist);
  
  switch (IRCode) {
    case 3: // Stop motors
      Serial.println("Stopping motors");
      break;
    case 2: // Medium-high speed
      Serial.println("Medium speed motors");
      break;
    case 0: // Speed & buzzer based on radars
      Serial.println("Changing motors based on radars");
      if (obstacleDist < 15.0) {
        receiverIR.stopTimer(); // tone() requires the same timer as the IR remote
        tone(BUZZER_PIN, alert_tone);
        delay(250);
        noTone(BUZZER_PIN);
        receiverIR.restartTimer(); // so the resource must be shared
      }
      break;
    default: // Idle speed
      Serial.println("Idling motors");
      break;
  }
}