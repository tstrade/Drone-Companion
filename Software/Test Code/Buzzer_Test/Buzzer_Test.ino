#include "pitches.h"

#define BUZZER_PIN 8

const int gameTones[] = { NOTE_G5, 
                          NOTE_FS5, 
                          NOTE_B5, 
                          NOTE_E5,
                          NOTE_D5,
                          NOTE_G5,
                          NOTE_C4,
                          NOTE_B4,
                          NOTE_E5,
                          NOTE_A4,
                          NOTE_D5};

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
