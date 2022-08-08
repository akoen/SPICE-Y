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
    // HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    Serial.println("Serial connected");
}
void pwmServoTest();
void loop() {
    
    // Servos::bridgeServo.write(Servos::bridge_open_angle);
    // delay (3000);
    // Servos::bridgeServo.write(Servos::bridge_closed_angle);
    // delay (1000);
    // Serial.println("loop");
    
    // float mags[2];
    // IR::getMagnitude(mags);
    // int goodReadings = 0;
    // int reqGoodReadings = 10;
    // while (goodReadings < reqGoodReadings) {
    //     if (Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT) < TreasureDetection::treasure_in_v_dist + TreasureDetection::treasure_in_claw_dist_err) {
    //         goodReadings++;
    //     } else {
    //         goodReadings = 0;
    //     }
    //     IR::driveWithPID();
    // }
    // int driveDuty = 40;
    // Encoders::driveMotorsDistance(driveDuty, true, 20, 2);
    // // while (true) {
    // //     Motors::driveBack(40);
    // // }
    // // back up a bit and turn right 90 degs and drive until edge detected
    // double firstTurnDeg = 60;
    // double secondTurnDeg = 30;

    // Encoders::driveMotorsDistance(30, true, 5);
    // delay(500);
    // Encoders::driveMotorsDistance(30, false, 5);
    // delay(500);
    // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, firstTurnDeg, 1.5);
    // Encoders::driveMotorsDistance(driveDuty, true, 8);
    // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BACKWARDS, secondTurnDeg, 1.5);
    
    // Encoders::driveMotorsDistance(driveDuty, false, 35, 2);
    // Servos::deployBridge();
    // obtain third treasure using IR PID
    // double driveFwdCm = 45;
    // double rotateLeftDegs = 100;
    // int driveDuty = 40;
    // bool cacheThirdTreasure = false;
    // TreasureDetection::obtainThirdIRtreasure(driveFwdCm, rotateLeftDegs, driveDuty, cacheThirdTreasure);

    Executor::execute();

    /////////
    //     float val = 0;
    //     float alpha = 0.1;
    // while (true) {
    //     val = alpha * Sonars::getDistanceSinglePulse(Sonars::SonarType::LEFT) + (1-alpha) * val;
    //     IR::driveWithPID();
    //     // Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
        
    //     Serial.println(val);
    // }
    /////////

    // Serial.print(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
    // Serial.print(" ");
    // Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT));
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
