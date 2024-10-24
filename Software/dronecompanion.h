#ifndef DRONECOMPANION_H
#define DRONECOMPANION_H

/*
This header file will contain all declarations necessary for our own library, consisting of
functions that are not provided by any given component's library.
*/

// Include all libraries needed for hardware
#include <stdio.h>
#include <stdlib.h>
#include <pitches.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// BMP280 Macros
const int BMP_SCK = 13;
const int BMP_MISO  = 12;
const int BMP_MOSI  = 11;
const int BMP_CS = 10;
const float STD_SEA_LEVEL_HPA = 1013.25;
const float DRONE_UPPER_LIM = 274.32;
const float DRONE_LOWER_LIM = 152.40;

// Adafruit Objects
Adafruit_BMP280 bmp(BMP_CS);


// BMP280 ground-level altitude
float ground_ref;

// Buzzer Macros
const int BUZZER_PIN = 8;
const int BUZZER_TONES = 6;

// Radar Macros
const int trigPin1 = 3;
const int echoPin1 = 2;
const int trigPin2 = 5;
const int echoPin2 = 4;
const int OBSTACLE_IN_RANGE = 200;
const int USER_OUT_OF_RANGE = 100;
const int USER_TOO_CLOSE = 80;
const int HC_1 = 0;
const int HC_2 = 1;

// Radar dist/dur initialization
long duration[2];
long distance[2];

// Radar Objects
Servo hc1, hc2;

// Buzzer Notes
const int alarm[] = { NOTE_B4,
                      NOTE_F4,
                      NOTE_B4,
                      NOTE_F4,
                      NOTE_B4,
                      NOTE_F4};

// Function Declarations
int init_takeoff();

void init_landing();

int calculate_distance(int trig, int echo, int radar_idx);

void process_radar();

void react_to_obstacle(int distance);

void react_to_user(int distance);

#endif
