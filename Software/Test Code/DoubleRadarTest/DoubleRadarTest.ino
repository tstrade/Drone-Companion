// Double Radar Functionality
#include <Servo.h>

const unsigned int trigFront = 14;
const unsigned int echoFront = 15;

const unsigned int trigBack = 16;
const unsigned int echoBack = 17;

// Out of range is 13+ feet (23200 us pulse)
const unsigned int MAX_DIST = 23200;

Servo frontRadar, backRadar;

unsigned long sendPulse(int trig, int echo) {
  unsigned long t0;
  //unsigned long tf;

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  /*
  while ( digitalRead(echo) == 0 );
  t0 = micros();

  while ( digitalRead(echo) == 0 );
  tf = micros();

  if ((tf - t0) >= MAX_DIST) { return MAX_DIST; }
  return (tf - t0);
  */
  t0 = pulseIn(echo, HIGH);
  if (t0 >= MAX_DIST) { return MAX_DIST; }
  return t0;
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

/* Questions/Notes for later:
 * if the speed function has passed, will the motors still spin? 
    - Ex: turn on all motors, then check for radar distance. will the motors stay on during the radar check?
 * (not to worry about now): How do we check both radars at the same time?
*/
/* The following is code for the demo only. this will not follow everything regarding obstacle and person detection, rather it will make the drone follow an object from radar 1 */
void loop() {
  frontRadar.write(trigFront);
  int pulseWidth = sendPulse(trigFront, echoFront);
  delay(500);
  switch (pulseWidth) {
    case MAX_DIST:
      Serial.println("Out of range from Front");
      break;
    default:
      Serial.print("Object is ");
      Serial.print(pulseWidth / 148.0);
      Serial.println(" inches away from front radar.");
      break;
  }
  backRadar.write(trigBack);
  pulseWidth = sendPulse(trigBack, echoBack);
  delay(500);
  switch (pulseWidth) {
    case MAX_DIST:
      Serial.println("Out of range from Back");
      break;
    default:
      Serial.print("Object is ");
      Serial.print(pulseWidth / 148.0);
      Serial.println(" inches away from back radar.");
      break;
  }
  delay(500);


  /*"standard speed"(will update)
    ESC1.speed(1500);
    ESC2.speed(1500);
    ESC3.speed(1500);
    ESC4.speed(1500);

    radar 1 will be pointing at user(side of motors 3 and 4)
    radar value will be calculated into a distance

    Radarvalue1 = calculateDistance(trigPin1, echoPin1);

    there will be a distance we want the drone to be from the user. the distance calculated will be compared to this distance.//
    here, if the distance is farther than we'd like, motors 1 and 2(which are facing away from the user) will increase in speed, moving the drone closer to the user.//
    while(Radarvalue1 > specified distance){
      ESC1.speed(2000);
      ESC2.speed(2000);
    }
    once the prefered distance has been reached, motors 1 and 2 will spin at normal speed causing it to stop moving.//
    ESC1.speed(1500);
    ESC2.speed(1500);
  
    If the drone is closer to the user than we'd like, motors 3 and 4 will increase in speed to move the drone away from the user//
    while(Radarvalue1 < specified distance){
      ESC3.speed(2000);
      ESC4.speed(2000);
    }
    once the prefered distance has been met, motors 3 and 4 will spin normally causing the drone to stop moving and remain in place.
    ESC3.speed(1500);
    ESC4.speed(1500);
  }
  
  
  (for later): radar 2 will be pointing awy from user, at objects(side of motors 1 and 2)
  */

}
