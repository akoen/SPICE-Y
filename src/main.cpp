#include "Arduino.h"
#include "HardwareTimer.h"

#include <cmath>

#include "config.h"
#include "helpers.h"
#include "board-setup.h"

/* Private define ------------------------------------------------------------*/


// the setup routine runs once when you press reset:
void setup() {
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
}


float magAvg0 = 0;
float magAvg1 = 0;
float alpha = 0.2;
void loop() {
  if (DMA1DataAvailable)
  {
    uint16_t pin0[IR_SENS_NUM_READINGS / 2] = {0};
    uint16_t pin1[IR_SENS_NUM_READINGS / 2] = {0};

    uint16_t *source[] = {pin0, pin1};

    deinterleave(DMA1Data, source, IR_SENS_NUM_READINGS, 2);

    // 12.5 cycles base rate + 71.5 cycles configurable divided by 2 channels
    const int sampleFreq = (ADC_CLOCK_SPEED_HZ / (12.5 + 71.5)) / 2;

    float mag0 = goertzelMagnitude(IR_SENS_NUM_READINGS / 2, 10000, sampleFreq, pin0);
    magAvg0 = alpha * mag0 + (1 - alpha) * magAvg0;

    float mag1 = goertzelMagnitude(IR_SENS_NUM_READINGS / 2, 10000, sampleFreq, pin1);
    magAvg1 = alpha * mag1 + (1 - alpha) * magAvg1;

    while (!Serial); Serial.write((uint8_t *)&magAvg0, 4);
    while (!Serial); Serial.write((uint8_t *)&magAvg1, 4);

    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    DMA1DataAvailable = false;
  }
}