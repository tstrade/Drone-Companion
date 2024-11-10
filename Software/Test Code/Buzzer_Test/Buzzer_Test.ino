#include "pitches.h"

#define BUZZER_PIN 8

const int gameTones[] = { NOTE_G4, 
                          NOTE_A4, 
                          NOTE_E4, 
                          NOTE_D4};

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  int i;
  for (i = 0; i < 4; i++) {
    tone(BUZZER_PIN, gameTones[i]);
    delay(300);
    noTone(BUZZER_PIN);
    delay(300);
  }
}
