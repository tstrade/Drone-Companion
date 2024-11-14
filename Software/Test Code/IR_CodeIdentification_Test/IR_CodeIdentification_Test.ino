#include <IRremote.h>

// Initialize IR receiver object on pin 7
IRrecv receiverIR(7);

void setup() {
  receiverIR.enableIRIn();
  Serial.begin(9600);
}

void loop() {
  if  (receiverIR.decode()) {
    Serial.println(receiverIR.decodedIRData.command);
    receiverIR.resume();
  }
}

/*
0 = bright up
1 = bright down
2 = ON
3 = OFF

4 = RED
5 = GREEN
6 = BLUE
7 = WHITE

8 = ORANGE
9 = YELLOW
10 = CYAN
11 = PURPLE

12 = J3
13 = J7
14 = F3
15 = F7

16 = NOTE1
17 = NOTE2
18 = NOTE3
19 = NOTE4
*/