#ifndef SPICEBOT_HELPERS_H
#define SPICEBOT_HELPERS_H

#include <stdint.h>

// Template - easiest to define in header file
template <class T> void deinterleave(T source[], T *dest[], uint16_t sourceLength, uint16_t numDest) {
    for(int i = 0; i < sourceLength/numDest; i ++) {
        for(int j = 0; j < numDest; j++) {
            dest[j][i] = source[i*numDest+j];
        }
    }
}

float goertzelMag(int numSamples, int targetFreq, int sampleFreq, uint16_t data[]);

#endif //SPICEBOT_HELPERS_H
