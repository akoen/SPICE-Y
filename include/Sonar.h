#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>

#define TRIG_PIN PB11
#define ECHO_PIN PB10
#define SONAR_MEAS_DELAY 70 // milliseconds before taking another sonar measurement

extern double *pDistance; // pointer to distance value

void setupSonar();
void ISR_GetDistance();

#endif