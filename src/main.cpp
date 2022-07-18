#include "Arduino.h"
#include "HardwareTimer.h"

#include <cmath> 

#include <config.h>
#include <board-setup.h>

/* Private define ------------------------------------------------------------*/

template <class T> void deinterleave(T source[], T *dest[], uint16_t sourceLength, uint16_t numDest) {
  for(int i = 0; i < sourceLength/numDest; i ++) {
      for(int j = 0; j < numDest; j++) {
        dest[j][i] = source[i*numDest+j];
      }
  }
}

float goertzel_mag(int numSamples, int targetFreq, int sampleFreq, uint16_t data[])
{
    float k = (numSamples * ((float) targetFreq / sampleFreq));
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

    // float scalingFactor = numSamples / 2.0;

    // Second stage
    float real = q1 - q2 * cosine;
    float imag = q2 * sine;

    float magnitude = sqrtf(real*real + imag*imag);
    return magnitude;
}

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB0, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);

  Serial.begin(115200, SERIAL_8N1);

  timerHeartbeatInit();

  /* Configure the ADC peripheral */
  ADC_Config();

  /* Run the ADC calibration */
  HAL_ADCEx_Calibration_Start(&AdcHandle);

  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
  // HAL_TIM_Base_Start_IT(timerSensorReadFast.getHandle());


}

constexpr uint32_t servoMG90SPercentToDuty(float p) {
return (uint32_t)((p * 0.05 + 0.05) * 4095);
}

float magAvg = 0;
float alpha = 0.2;
void loop()
{
  // pwm_start(PB_6, 50, servoMG90SPercentToDuty(0.1), RESOLUTION_12B_COMPARE_FORMAT);
  // // pwm_start(PB_6, 50, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  // delay(1000);
  // pwm_start(PB_7, 50, servoMG90SPercentToDuty(0.9), RESOLUTION_12B_COMPARE_FORMAT);
  // delay(1000);
  // pwm_start(PB_6, 50, servoMG90SPercentToDuty(0.9), RESOLUTION_12B_COMPARE_FORMAT);
  // delay(1000);
  // pwm_start(PB_7, 50, servoMG90SPercentToDuty(0.1), RESOLUTION_12B_COMPARE_FORMAT);
  // delay(1000);

  if(DMA1DataAvailable) {
    digitalWrite(PB0, HIGH);

    uint16_t pin1[IR_SENS_NUM_READINGS/2] = {0};
    uint16_t pin2[IR_SENS_NUM_READINGS/2] = {0};

    uint16_t *source[] = {pin1, pin2};

    deinterleave<uint16_t>(DMA1Data, source, IR_SENS_NUM_READINGS, 2);

    float mag = goertzel_mag(IR_SENS_NUM_READINGS/2, 10000, (12000000/(12.5+71.5))/2, pin1);
    magAvg = alpha * mag + (1-alpha) * magAvg;
    while (!Serial);
    // uint8_t tmp = 1000;
    // Serial.write((uint8_t *) pin1, sizeof(pin1));
    // delay(1000);
    Serial.write((uint8_t *) &magAvg, 4);
    // while (!Serial);
    // Serial.write((uint8_t *) pin2, sizeof(pin2));
    
    digitalWrite(PB0, LOW);
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    DMA1DataAvailable = false;
  }
}