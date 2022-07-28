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

// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200, SERIAL_8N1);
    Setup::timerHeartbeatInit();
    Setup::ADC();

    Motors::configMotorPins();
    Encoders::configEncoderPins();
    Sonars::configSonarPins();
    ReflectanceSensors::configFrontReflectanceSensors();
    // pinMode(PB9, OUTPUT);
    Servos::configArmClawPins();
    // pinMode(BRIDGE_SERVO_PIN, OUTPUT);
    // pinMode(BOX_SERVO_PIN, OUTPUT);
    // pinMode(CLAW_SERVO_PIN, OUTPUT);
    // pinMode(CLAW_SERVO_PIN, OUTPUT);

    /* Run the ADC calibration */
    HAL_ADCEx_Calibration_Start(&AdcHandle);

    // /* Start ADC conversion on regular group with transfer by DMA */
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
}

void loop() {
    // pwm_start(PB_9, 100, (int)(0.5*4096), RESOLUTION_12B_COMPARE_FORMAT);
    // ReflectanceSensors::readFrontReflectanceSensors();
    // ReflectanceSensors::printFrontReflectance();
    TreasureDetection::obtainFirstTreasure();
    // Servos::clawServo.write(60);
    // delay(2000);
    // Servos::clawServo.write(180);
    // delay(1000);
    // Motors::drive();

    
}

void pwmServoTest() {
    // Servos::setServoPos(Servos::clawServo, Servos::claw_close_angle);
    // pwm_start(CLAW_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.5/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BRIDGE_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.5/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BOX_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.5/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // delay(1000);
    // pwm_start(CLAW_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BRIDGE_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BOX_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // delay(1000);
    // pwm_start(CLAW_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BRIDGE_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(BOX_SERVO_PIN_PWM_FORMAT, 50, (uint32_t)(4096 * (1.8/20)), RESOLUTION_12B_COMPARE_FORMAT);
    // delay(1000);
}
