#include "servo-controller.h"
#include "magnetic-sensor.h"

const int Servos::claw_close_angle = 50;
const int Servos::claw_part_open_angle = 100;
const int Servos::claw_full_open_angle = 180;
const int Servos::claw_bomb_detect_angle = 110;
const int Servos::arm_bomb_detect_angle = 110;
const int Servos::arm_lowered_angle = 0;
const int Servos::arm_lifted_angle = 70;

Servo Servos::clawServo;
Servo Servos::armServo;
Servo Servos::bridgeServo;
Servo Servos::boxServo;

bool Servos::pinsConfiged = false;

void Servos::configServoPins() {
    if (pinsConfiged) return;
    
    clawServo.attach(CLAW_SERVO_PIN);
    armServo.attach(ARM_SERVO_PIN);
    bridgeServo.attach(BRIDGE_SERVO_PIN);
    boxServo.attach(BOX_SERVO_PIN);
}

void Servos::collectTreasure() {
    // open claw as we lower arm
    clawServo.write(claw_part_open_angle);

    // hall effect bomb detect condition
    if (!BombDetection::bombEncounteredFlag) {
        armServo.write(arm_bomb_detect_angle);
        delay(1000);
        clawServo.write(claw_bomb_detect_angle);
        delay(1000);
        if (!BombDetection::isBombDetected) {
            clawServo.write(claw_close_angle);
        } else {
            // don't pick up - bomb found
            BombDetection::bombEncounteredFlag = true;
        }
    } else {
        armServo.write(arm_lowered_angle);    
        delay(1000);
        clawServo.write(claw_close_angle);
    }
    delay(1000);

    //lift arm  
    armServo.write(arm_lifted_angle);
    delay(1000);

    // partially open claw
    clawServo.write(claw_part_open_angle);
    delay(1000);
}
