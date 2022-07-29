#include "ir.h"
#include "board-setup.h"
#include "helpers.h"
#include "motor-driver.h"

namespace IR {
    const int targetFrequency = 10000;
    const float sampleFrequency = (12000000 / (12.5 + 71.5)) / 2;
    const float kp = 20;
    const float kd = 8;

    float magSmoothed[] = {0, 0};
    float prevP = 0;

    void getMagnitude(float magnitude[]) {
        uint16_t dataL[IR_SENS_NUM_READINGS / 2] = {0};
        uint16_t dataR[IR_SENS_NUM_READINGS / 2] = {0};
        uint16_t *source[] = {dataL, dataR};

        deinterleave<uint16_t>(DMA1Data, source, IR_SENS_NUM_READINGS, 2);
        magnitude[0] = goertzelMag(IR_SENS_NUM_READINGS / 2, targetFrequency, sampleFrequency, dataL);
        magnitude[1] = goertzelMag(IR_SENS_NUM_READINGS / 2, targetFrequency, sampleFrequency, dataR);

        while(!Serial);
        Serial.write((uint8_t *) magnitude, 8);
    }

    float calcPID() {
        float magnitude[2];
        getMagnitude(magnitude);

        float p = (magnitude[1] - magnitude[0]) / 100;
        float d = p - prevP;

        prevP = p;

        return kp*p + kd*d;
    }

    void driveWithPID() {
        if (DMA1DataAvailable) {
            float pid = calcPID();
            // while(!Serial);
            // Serial.write((uint8_t *) &pid, 4);
            Motors::setDutyCycles(Motors::dutyCycleL + pid, Motors::dutyCycleR - pid);
            Motors::setDir(true, true);
            Motors::drive();

            DMA1DataAvailable = false;
            HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
        }

    }
}
