#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>

#define TRIG_PIN PB11
#define ECHO_PIN PB10
#define SONAR_MEAS_DELAY 70 //milliseconds beforetaking another sonar measurement

double* pDistance; //pointer to distance

void setupSonar();
void getDistance();


#endif