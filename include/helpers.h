#ifndef INCLUDE_HELPERS
#define INCLUDE_HELPERS

#include <stdint.h>

constexpr uint32_t servoMG90SPercentToDuty(float p);

float goertzelMagnitude(int numSamples, int targetFreq, int sampleFreq, uint16_t data[]);

void deinterleave(uint16_t source[], uint16_t *dest[], uint16_t sourceLength, uint16_t numDest);


#endif