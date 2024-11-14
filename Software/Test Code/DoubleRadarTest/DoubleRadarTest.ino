// Double Radar Functionality
#include <Servo.h>

const unsigned int trigFront = 16;
const unsigned int echoFront = 17;

const unsigned int trigBack = 14;
const unsigned int echoBack = 15;

// Out of range is 13+ feet (23200 us pulse)
const unsigned int MAX_DIST = 23200;

Servo frontRadar, backRadar;

int calculateDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.034/2);
}

void setup() {
  //setup radars pinout and input reading
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  digitalWrite(trigFront, LOW);

  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);
  digitalWrite(trigBack, LOW);

  Serial.begin(9600);
}

void loop() {
  frontRadar.write(trigFront);
  int frontDist = calculateDistance(trigFront, echoFront);
  delay(500);
  switch (frontDist) {
    case MAX_DIST:
      Serial.println("Max Distance or Out of range from Front sensor");
      break;
    default:
      Serial.print("Object is ");
      Serial.print(frontDist);
      Serial.println(" centimeters away from front radar sensor.");
      break;
  }

  backRadar.write(trigBack);
  int backDist = calculateDistance(trigBack, echoBack);
  delay(500);
  switch (backDist) {
    case MAX_DIST:
      Serial.println("Max Distance or Out of range from Back sensor");
      break;
    default:
      Serial.print("Object is ");
      Serial.print(backDist);
      Serial.println(" centimeters away from back radar sensor.");
      break;
  }
  delay(500);
}
