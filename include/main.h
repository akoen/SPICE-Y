#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "SpiceServo.h"
#include "Sonar.h"

#define LED_BUILTIN PC13
#define SERIAL_BAUD 9600
#define TIMER1_OVERFLOW 200000 // microseconds

typedef enum
{
    CHANNEL1 = 1,
    CHANNEL2 = 2,
    CHANNEL3 = 3,
    CHANNEL4 = 4
} timerChannels_t;

extern timerChannels_t channel1;
extern timerChannels_t channel2;
extern timerChannels_t channel3;
extern timerChannels_t channel4;
extern double newDistance;
extern double previousDistance;

// function prototypes
void ISR_Heartbeat();

#endif