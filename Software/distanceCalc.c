#define X 0
#define Y 1
#define Z 2
#define D 3
#define NRANGE -0.05
#define PRANGE 0.05


static float userCoordinates[D] = {0, 0, 0};
float coordinateDifference[D];

float checkX(int droneCoordinates[]) {
  return droneCoordinates[X] - userCoordinates[X];
}

float checkY(int droneCoordinates[]) {
  return droneCoordinates[Y] - userCoordinates[Y];
}

float checkZ(int droneCoordinates[]) {
  return droneCoordinates[Z] - userCoordinates[Z];
}

float compareCoords(float coordinate, int dir) {
  return coordinate - userCoordinates[dir];
}

float *computeDifference(float droneCoordinates[]) {
  coordinateDifference[X] = compareCoords(droneCoordinates[X], X);
  coordinateDifference[Y] = compareCoords(droneCoordinates[Y], Y);
  coordinateDifference[Z] = compareCoords(droneCoordinates[Z], Z);
  return coordinateDifference;
}

int aligned(float difference) {
  if (difference < NRANGE || difference > PRANGE) {
    return 0;
  }
  return 1;
}


float readjust(float coordinateDifference[]) {
  while (aligned(coordinateDifference[X]) ||
         aligned(coordinateDifference[Y]) ||
         aligned(coordinateDifference[Z])) {
    // adjust coordinate with biggest difference?
    // or just adjust in order of importance:
    // Y (in case of turning)
    // X (too far or too close)
    // Z (too high or too low)
    /* Arduino code will probably look something like:
       adjustY();
       adjustX();
       adjustZ();
       ... and so on. Could also change importance
       by calling one dimension more often than the others
    */
  }
  return 0.0;
}
