#include "servo-controller.h"
#include "magnetic-sensor.h"

const int Servos::claw_close_angle = 60;
const int Servos::claw_part_open_angle = 100;
const int Servos::claw_full_open_angle = 180;
const int Servos::claw_bomb_detect_angle = 110;
const int Servos::arm_bomb_detect_angle = 70;
const int Servos::arm_lowered_angle = 15;   // parallel
const int Servos::arm_lifted_angle = 120;

Servo Servos::clawServo;
Servo Servos::armServo;
Servo Servos::bridgeServo;
Servo Servos::boxServo;

bool Servos::clawPinsConfiged = false;;
bool Servos::armPinsConfiged = false;
bool Servos::bridgePinsConfiged = false;
bool Servos::boxPinsConfiged = false;

void Servos::configAllServoPins() {
    if (!clawPinsConfiged) {
        clawServo.attach(CLAW_SERVO_PIN);
        clawPinsConfiged = true;
    }
    if (!armPinsConfiged) {
        armServo.attach(ARM_SERVO_PIN);
        armPinsConfiged = true;
    }
    if (!bridgePinsConfiged) {
        bridgeServo.attach(BRIDGE_SERVO_PIN);
        bridgePinsConfiged = true;
    }
    if (!boxPinsConfiged) {
        boxServo.attach(BOX_SERVO_PIN);
        boxPinsConfiged = true;
    }
}

void Servos::configArmClawPins() {
    if (!clawPinsConfiged) {
        clawServo.attach(CLAW_SERVO_PIN);
        clawPinsConfiged = true;
    }
    if (!armPinsConfiged) {
        armServo.attach(ARM_SERVO_PIN);
        armPinsConfiged = true;
    }
}

void Servos::collectTreasure() {
    // open claw as we lower arm
    clawServo.write(claw_part_open_angle);
    delay(100);
    clawServo.write(claw_full_open_angle);
    delay(2000);
    // hall effect bomb detect condition
    if (!BombDetection::bombEncounteredFlag) {
        armServo.write(arm_bomb_detect_angle);
        delay(2000);
        clawServo.write(claw_bomb_detect_angle);
        delay(2000);
        if (!BombDetection::isBombDetected) {
            clawServo.write(claw_close_angle);
        } else {
            // don't pick up - bomb found
            BombDetection::bombEncounteredFlag = true;
        }
    } else {
        armServo.write(arm_lowered_angle);    
        delay(2000);
        clawServo.write(claw_close_angle);
    }
    delay(2000);

    //lift arm  
    armServo.write(arm_lifted_angle);
    delay(2000);

    // partially open claw
    clawServo.write(claw_part_open_angle);
    delay(2000);
}

void Servos::setServoPos(Servo servoObj, int deg) {
    servoObj.write(deg);
}