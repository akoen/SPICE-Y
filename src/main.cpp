#include <Arduino.h>
#include <HardwareTimer.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "motor-driver.h"
#include "tape-follower.h"
#include "encoder.h"
#include "helpers.h"
#include "board-setup.h"

using namespace Motors;
using namespace Encoders;


/**
 * @brief Sets up the OLED display
 */


// the setup routine runs once when you press reset:
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PB0, OUTPUT);

    Serial.begin(115200, SERIAL_8N1);

    Setup::timerHeartbeatInit();
    Setup::ADC();
    Setup::OLED();

    configMotorPins();
    configEncoderPins();

      /* Run the ADC calibration */
      HAL_ADCEx_Calibration_Start(&AdcHandle);

      /* Start ADC conversion on regular group with transfer by DMA */
      HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
}

void tapeFollowingPidTest();

float magAvg = 0;
volatile double refTime = 0;
volatile double currTime = 0;
void loop() {

    tapeFollowingPidTest();

    if (DMA1DataAvailable) {
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

        HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *) DMA1Data, IR_SENS_NUM_READINGS);
        DMA1DataAvailable = false;
    }
}

void tapeFollowingPidTest() {
    OLEDDisplayHandler.clearDisplay();
    OLEDDisplayHandler.setCursor(0, 0);

    TapeFollow::driveWithPid();

    // ReflectanceSensors::readFrontReflectanceSensors();
    // Motors::drive();

    OLEDDisplayHandler.print("Sensors(L,M,R): ");
    OLEDDisplayHandler.print(TapeFollow::onTapeL);
    OLEDDisplayHandler.print(" ");
    OLEDDisplayHandler.print(TapeFollow::onTapeM);
    OLEDDisplayHandler.print(" ");
    OLEDDisplayHandler.println(TapeFollow::onTapeR);

    // OLEDDisplayHandler.print(ReflectanceSensors::frontSensorLval);
    // OLEDDisplayHandler.print(" ");
    // OLEDDisplayHandler.print(ReflectanceSensors::frontSensorMval);
    // OLEDDisplayHandler.print(" ");
    // OLEDDisplayHandler.print(ReflectanceSensors::frontSensorRval);


    OLEDDisplayHandler.print("PWM change: ");
    OLEDDisplayHandler.println(TapeFollow::pwmChange);
    OLEDDisplayHandler.print("Duty(L,R): ");
    OLEDDisplayHandler.print(Motors::dutyCycleL);
    OLEDDisplayHandler.print(" ");
    OLEDDisplayHandler.println(Motors::dutyCycleR);
    OLEDDisplayHandler.display();
}
