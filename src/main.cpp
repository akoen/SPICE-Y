#include <Arduino.h>
#include <HardwareTimer.h>

#include "motor-driver.h"
#include "tape-follower.h"
#include "encoder.h"
#include "helpers.h"
#include "board-setup.h"
#include "ir.h"
#include "treasure-detection.h"
#include "servo-controller.h"
#include "executor.h"

/* the setup routine runs once when you press reset */
void setup() {
    Serial.begin(115200, SERIAL_8N1);
    Setup::timerHeartbeatInit();
    Setup::ADC();

    Motors::configMotorPins();
    Encoders::configEncoderPins();
    Sonars::configSonarPins();
    ReflectanceSensors::configFrontReflectanceSensors();
    Servos::configAllServoPins();
    
    /* Run the ADC calibration */
    HAL_ADCEx_Calibration_Start(&AdcHandle);
    pinMode(PA12, OUTPUT);
    /* Start ADC conversion on regular group with transfer by DMA */
    // HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    Serial.println("Serial connected");
}

void loop() {
    Executor::execute();
    exit(0);
}
