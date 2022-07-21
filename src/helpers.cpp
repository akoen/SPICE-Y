#include <helpers.h>
#include <config.h>

#include <cmath>

constexpr uint32_t servoMG90SPercentToDuty(float p) {
  return (uint32_t)((p * 0.05 + 0.05) * 4095);
}


// Splits inteleaved source array into multiple arrays
void deinterleave(uint16_t source[], uint16_t *dest[], uint16_t sourceLength, uint16_t numDest) {
  for (int i = 0; i < sourceLength / numDest; i++) {
    for (int j = 0; j < numDest; j++) {
      dest[j][i] = source[i * numDest + j];
    }
  }
}

float goertzelMagnitude(int numSamples, int targetFreq, int sampleFreq, uint16_t data[]) {
  float k = (numSamples * ((float)targetFreq / sampleFreq));
  float omega = (2.0 * PI * k) / numSamples;
  float sine = sin(omega);
  float cosine = cos(omega);
  float q0 = 0;
  float q1 = 0;
  float q2 = 0;

  // First stage
  for (int i = 0; i < numSamples; i++)
  {
    q0 = 2.0 * cosine * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }

  // float scalingFactor = numSamples / 2.0;

  // Second stage
  float real = q1 - q2 * cosine;
  float imag = q2 * sine;

  float magnitude = sqrtf(real * real + imag * imag);
  return magnitude;
}