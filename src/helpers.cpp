//
// Created by alex on 22/07/22.
//

#include <board-setup.h>
#include <config.h>
#include <cmath>
#include "HardwareTimer.h"
#include "Arduino.h"
#include "helpers.h"

float goertzelMag(int numSamples, int targetFreq, int sampleFreq, uint16_t data[])
{
    int k = (int) (0.5 + numSamples * ((float) targetFreq / sampleFreq));
    float omega = (2.0 * PI * k) / numSamples;
    float sine = sin(omega);
    float cosine = cos(omega);
    float q0=0;
    float q1=0;
    float q2=0;

    // First stage
    for(int i=0; i<numSamples; i++)
    {
        q0 = 2.0 * cosine * q1 - q2 + data[i];
        q2 = q1;
        q1 = q0;
    }

    float scalingFactor = numSamples / 2.0;

    // Second stage
    float real = (q1 - q2 * cosine) / scalingFactor;
    float imag = (q2 * sine) / scalingFactor;

    float magnitude = sqrtf(real*real + imag*imag);
    return magnitude;
}