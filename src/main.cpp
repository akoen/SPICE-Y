#include "Arduino.h"
#include "HardwareTimer.h"
#include "helpers.h"

#include <config.h>
#include <board-setup.h>

// the setup routine runs once when you press reset:
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB0, OUTPUT);

  Serial.begin(115200, SERIAL_8N1);

  Setup::timerHeartbeatInit();
  Setup::ADC();

  /* Run the ADC calibration */
  HAL_ADCEx_Calibration_Start(&AdcHandle);

  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
}

float magAvg = 0;
void loop() {
  if(DMA1DataAvailable) {
    digitalWrite(PB0, HIGH);

    uint16_t pin1[IR_SENS_NUM_READINGS/2] = {0};
    uint16_t pin2[IR_SENS_NUM_READINGS/2] = {0};

    uint16_t *source[] = {pin1, pin2};

    deinterleave<uint16_t>(DMA1Data, source, IR_SENS_NUM_READINGS, 2);

    float mag = goertzelMag(IR_SENS_NUM_READINGS / 2, 10000, 14000000 / (12.5 + 71.5), pin1);
    magAvg = IR_SMOOTHING_ALPHA * mag + (1 - IR_SMOOTHING_ALPHA) * magAvg;
    while (!Serial);
    // Serial.write((uint8_t *) pin1, sizeof(pin1));
    Serial.write((uint8_t *) &magAvg, 4);
    // while (!Serial);
    // Serial.write((uint8_t *) pin2, sizeof(pin2));
    
    digitalWrite(PB0, LOW);
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    DMA1DataAvailable = false;
  }
}