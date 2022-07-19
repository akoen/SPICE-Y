#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>

#define TRIG_PIN PB11
#define ECHO_PIN PB10
#define SONAR_MEAS_DELAY 70 // milliseconds before taking another sonar measurement
#define SPEED_SOUND_MICS 0.034 //speed of sound in cm / microsecond

void setupSonar();
void ISR_SonarEcho();
void ISR_SonarTrigger();
void TriggerSonar();

extern volatile bool readyToTrigger;
extern volatile bool pulseCaptured;
extern volatile uint32_t pulseDuration;

#endif