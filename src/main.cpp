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
    // Serial.begin(115200, SERIAL_8N1);
    // Setup::timerHeartbeatInit();
    // Setup::ADC();

    Motors::configMotorPins();
    Encoders::configEncoderPins();
    // Sonars::configSonarPins();
    // ReflectanceSensors::configFrontReflectanceSensors();
    // Servos::configArmClawPins();

    // /* Run the ADC calibration */
    // HAL_ADCEx_Calibration_Start(&AdcHandle);
    // pinMode(PA12, OUTPUT);
    // // /* Start ADC conversion on regular group with transfer by DMA */
    // HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    // Serial.println("Serial connected");
}
void pwmServoTest();
void loop() {
    Motors::MotorAction action = Motors::MotorAction::DRIVE_FWD;
    Motors::RotateMode rotate = Motors::RotateMode::NONE;
    int duty = LW_PWM_DUTY;
    Encoders::startAddActionCache(action, rotate, duty);
    Encoders::driveMotorsDistance(duty, true, 20);
    delay(10);
    Motors::stopWithBrake(action, rotate, LW_PWM_DUTY, 10);
    Encoders::endAddActionCache();
    Encoders::executeReverseCache(2000);
    // TapeFollow::driveWithPid();
    // Serial.print(HIGH);
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
