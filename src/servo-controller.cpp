#include "servo-controller.h"
#include "magnetic-sensor.h"

// pos when gripping treasure in claw 
const int Servos::claw_close_angle = 37;
// pos when dropping treasure and initial position when claw is lifted up. Must clear the beacon mounts
const int Servos::claw_part_open_angle = 100;  
// pos when opening claw to collect treasure
const int Servos::claw_full_open_angle = 200;
const int Servos::claw_bomb_detect_angle = 110;

const int Servos::arm_bomb_detect_angle = 78;
const int Servos::above_treasure_below_IR_angle = 120;
const int Servos::arm_lowered_angle = 60;   // parallel
const int Servos::arm_lifted_angle = 165;

const int Servos::box_closed_angle = 60;
const int Servos::box_open_angle = 10;

const int Servos::bridge_closed_angle = 65;
const int Servos::bridge_open_angle = 45;

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
        clawServo.write(claw_close_angle);
        clawPinsConfiged = true;
    }
    if (!armPinsConfiged) {
        armServo.attach(ARM_SERVO_PIN);
        armServo.write(arm_lifted_angle);
        armPinsConfiged = true;
    }
    if (!bridgePinsConfiged) {
        bridgeServo.attach(BRIDGE_SERVO_PIN);
        bridgeServo.write(bridge_closed_angle);
        bridgePinsConfiged = true;
    }
    if (!boxPinsConfiged) {
        boxServo.attach(BOX_SERVO_PIN);
        boxPinsConfiged = true;
        boxServo.write(box_closed_angle);
    }
}

void Servos::configArmClawPins() {
    if (!clawPinsConfiged) {
        clawServo.attach(CLAW_SERVO_PIN);
        clawServo.write(claw_close_angle);
        clawPinsConfiged = true;
    }
    if (!armPinsConfiged) {
        armServo.attach(ARM_SERVO_PIN);
        armServo.write(arm_lifted_angle);
        armPinsConfiged = true;
    }
}

void Servos::collectTreasure() {
    BombDetection::configMagneticSensorPin;

    // should be in lifted arm, partly closed claw position

    // open claw as we lower arm
    clawServo.write(claw_part_open_angle);
    delay(300);
    armServo.write(above_treasure_below_IR_angle);
    delay(300);
    clawServo.write(claw_full_open_angle);
    delay(500);
    // hall effect
    if (!BombDetection::bombEncounteredFlag) {
        armServo.write(arm_bomb_detect_angle);
        delay(300);
        clawServo.write(claw_bomb_detect_angle);
        delay(500);
        if (!BombDetection::isBombDetected()) {
            // bomb not found
            clawServo.write(claw_full_open_angle);
            delay(300);
            armServo.write(arm_lowered_angle);
            delay(500);    
            // check again
            if (BombDetection::isBombDetected()) {
                BombDetection::bombEncounteredFlag = true;
            } else {
                clawServo.write(claw_close_angle);
            }
        } else {
            // don't pick up - bomb found
            BombDetection::bombEncounteredFlag = true;
        }
    } else {
        armServo.write(arm_lowered_angle);    
        delay(500);
        clawServo.write(claw_close_angle);
    }
    delay(500);
    armServo.write(arm_lifted_angle);
    delay(500);
    clawServo.write(claw_part_open_angle);
    delay(300);
}

void Servos::deployBox() {
    boxServo.write(box_open_angle);
}

void Servos::deployBridge() {
    bridgeServo.write(box_open_angle);
}
void Servos::setServoPos(Servo servoObj, int deg) {
    servoObj.write(deg);
}