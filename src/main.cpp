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

// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200, SERIAL_8N1);
    Setup::timerHeartbeatInit();
    Setup::ADC();

    Motors::configMotorPins();
    Encoders::configEncoderPins();
    Sonars::configSonarPins();
    ReflectanceSensors::configFrontReflectanceSensors();
    // Servos::configArmClawPins();
    Servos::configAllServoPins();

    // /* Run the ADC calibration */
    HAL_ADCEx_Calibration_Start(&AdcHandle);
    pinMode(PA12, OUTPUT);
    // /* Start ADC conversion on regular group with transfer by DMA */
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    Serial.println("Serial connected");
}
void pwmServoTest();
void loop() {
    // TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS);
    // delay(1000);
    // Serial.println("loop");
    // TreasureDetection::obtainTapeTreasure(2);

    // Servos::collectTreasure();
    // pwmServoTest();
    Executor::execute();
    // Serial.print(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
    // Serial.print(" ");
    // Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT));
    // Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, true, Motors::RotateMode::BACKWARDS, 50);
    // delay(2000);
    // Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::BACKWARDS, 90);
    // delay(2000);

    // TapeFollow::driveWithPid();
    // TreasureDetection::obtainTapeTreasure(1);

    // int duty = 50;
    // Motors::setDir(false, false);
    // Motors::setDutyCycles(duty, duty);
    // Motors::drive();
    // Serial.println("loop");
    // Encoders::startAddActionCache(action, rotate, duty);
    // Encoders::driveMotorsDistance(duty, true, 20);
    // Encoders::driveMotorsDistance(duty, false, 20);
    // Encoders::endAddActionCache();
    // delay(1000);
    // Encoders::executeReverseCache(2000);
    // delay(2000);
    // TapeFollow::driveWithPid();
    // IR::driveWithPID();
}

void pwmServoTest() {
    Servos::setServoPos(Servos::clawServo, Servos::claw_full_open_angle);
    Servos::setServoPos(Servos::armServo, Servos::arm_lowered_angle);
    delay(1000);
    Servos::setServoPos(Servos::clawServo, Servos::claw_close_angle);
    delay(1000);
    Servos::setServoPos(Servos::armServo, Servos::arm_lifted_angle);
    delay(1000);
    Servos::setServoPos(Servos::clawServo, Servos::claw_part_open_angle);
    delay(500);
    Servos::setServoPos(Servos::armServo, Servos::arm_bomb_detect_angle);
    delay(1000);

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
