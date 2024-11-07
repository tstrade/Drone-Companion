//radar functionality//
#include <Servo.h>

#define trigpin1 3
#define echopin1 2

#define trigpin2 5
#define echopin2 4

long duration;
long distance;

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
  //setup radars pinout and input reading
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  Serial.begin(9600);
}

/* Questions/Notes for later:
 * if the speed function has passed, will the motors still spin? 
    - Ex: turn on all motors, then check for radar distance. will the motors stay on during the radar check?
 * (not to worry about now): How do we check both radars at the same time?
*/
/* The following is code for the demo only. this will not follow everything regarding obstacle and person detection, rather it will make the drone follow an object from radar 1 */
void loop() {
  //"standard speed"(will update)//
  // ESC1.speed(1500);
  // ESC2.speed(1500);
  // ESC3.speed(1500);
  // ESC4.speed(1500);

    //radar 1 will be pointing at user(side of motors 3 and 4)//
    //radar value will be calculated into a distance//
    Radarvalue1 = calculateDistance(trigPin1, echoPin1);
    //there will be a distance we want the drone to be from the user. the distance calculated will be compared to this distance.//
    //here, if the distance is farther than we'd like, motors 1 and 2(which are facing away from the user) will increase in speed, moving the drone closer to the user.//
    while(Radarvalue1 > /*specified distance*/){
      // ESC1.speed(2000);
      // ESC2.speed(2000);
    }
    // once the prefered distance has been reached, motors 1 and 2 will spin at normal speed causing it to stop moving.//
    //ESC1.speed(1500);
    //ESC2.speed(1500);
  
    //If the drone is closer to the user than we'd like, motors 3 and 4 will increase in speed to move the drone away from the user//
    while(Radarvalue1 < /*specified distance*/){
      // ESC3.speed(2000);
      // ESC4.speed(2000);
    }
    //once the prefered distance has been met, motors 3 and 4 will spin normally causing the drone to stop moving and remain in place.
    //ESC3.speed(1500);
    //ESC4.speed(1500);
  }
  
  
  //(for later): radar 2 will be pointing awy from user, at objects(side of motors 1 and 2)


}
