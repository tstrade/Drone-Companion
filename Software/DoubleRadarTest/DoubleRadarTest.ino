#include <Servo.h>

#define trigPin1 3
#define echoPin1 2

#define trigPin2 5
#define echoPin2 4

long duration;
long distance;

Servo hc1,hc2;

int calculateDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034/2;
  return distance;
}

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  Serial.begin(9600);
}

void loop() {
  int i;
  
  for (i=15; i<=165; i++) {
    hc1.write(i);
    delay(500);
    calculateDistance(trigPin1, echoPin1);
    Serial.print("Radar 1: ");
    Serial.print(distance);
    Serial.print("\n");

    hc2.write(i);
    delay(500);
    calculateDistance(trigPin2, echoPin2);
    Serial.print("Radar 2: ");
    Serial.print(distance);
    Serial.print("\n");
  }
}
