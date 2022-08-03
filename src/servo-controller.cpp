#include "servo-controller.h"
#include "magnetic-sensor.h"

// pos when gripping treasure in claw 
const int Servos::claw_close_angle = 50;
// pos when dropping treasure and initial position when claw is lifted up. Must clear the beacon mounts
const int Servos::claw_part_open_angle = 100;  
// pos when opening claw to collect treasure
const int Servos::claw_full_open_angle = 190;
const int Servos::claw_bomb_detect_angle = 110;

const int Servos::arm_bomb_detect_angle = 50;
const int Servos::above_treasure_below_IR_angle = 70;
const int Servos::arm_lowered_angle = 15;   // parallel
const int Servos::arm_lifted_angle = 129;

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
    BombDetection::configMagneticSensorPin;

    // should be in lifted arm, partly closed claw position

    // open claw as we lower arm
    clawServo.write(claw_part_open_angle);
    delay(500);
    armServo.write(above_treasure_below_IR_angle);
    delay(1200);
    clawServo.write(claw_full_open_angle);
    delay(1200);
    // hall effect
    if (!BombDetection::bombEncounteredFlag) {
        armServo.write(arm_bomb_detect_angle);
        delay(1000);
        clawServo.write(claw_bomb_detect_angle);
        delay(1000);
        if (!BombDetection::isBombDetected()) {
            // bomb not found
            clawServo.write(claw_full_open_angle);
            delay(1000);
            armServo.write(arm_lowered_angle);    
            delay(1500);
            clawServo.write(claw_close_angle);
        } else {
            // don't pick up - bomb found
            BombDetection::bombEncounteredFlag = true;
        }
    } else {
        armServo.write(arm_lowered_angle);    
        delay(1500);
        clawServo.write(claw_close_angle);
    }
    delay(1000);

    armServo.write(arm_lifted_angle);
    delay(1000);

    clawServo.write(claw_part_open_angle);
    delay(1000);
}

void Servos::setServoPos(Servo servoObj, int deg) {
    servoObj.write(deg);
}