#include "servo-controller.h"
#include "magnetic-sensor.h"

namespace Servos {
    const int claw_close_angle = 6;
    const int claw_part_open_angle = 60;  
    const int claw_full_open_angle = 150;
    const int claw_bomb_detect_angle = 50;

    const int arm_bomb_detect_angle = 80;
    const int arm_part_lifted_angle = 120;
    const int arm_lowered_angle = 55;   // parallel
    const int arm_lifted_angle = 167;

    const int box_closed_angle = 53;
    const int box_open_angle = 0;

    const int bridge_closed_angle = 100;
    const int bridge_open_angle = 60;

    Servo clawServo;
    Servo armServo;
    Servo bridgeServo;
    Servo boxServo;

    bool clawPinsConfiged = false;
    bool armPinsConfiged = false;
    bool bridgePinsConfiged = false;
    bool boxPinsConfiged = false;

    void configAllServoPins() {
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

    void configArmClawPins() {
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
        BombDetection::configMagneticSensorPin(false);
    }

    void collectTreasure() {
        // should be in lifted arm, closed claw position

        // open claw as we lower arm
        armServo.write(arm_part_lifted_angle);
        delay(400);
        clawServo.write(claw_full_open_angle);
        delay(300);
        bool bombNowDetected = false;

        // bomb detection
        if (!BombDetection::bombEncounteredFlag) {
            armServo.write(arm_bomb_detect_angle);
            delay(300);
            clawServo.write(claw_bomb_detect_angle);
            delay(500);
            if (!BombDetection::isBombDetected()) {
                // bomb not found
                clawServo.write(claw_full_open_angle);
                delay(250);
                armServo.write(arm_lowered_angle);
                delay(500);
                clawServo.write(claw_part_open_angle);
                delay(250);
                // check again
                if (!BombDetection::isBombDetected()) {
                    clawServo.write(claw_close_angle);
                    delay(250);
                    if (BombDetection::isBombDetected()) {
                        bombNowDetected = true;
                    }
                } else {
                    bombNowDetected = true;
                }
                if (bombNowDetected) {
                    clawServo.write(claw_full_open_angle);
                    delay(200);
                    armServo.write(arm_part_lifted_angle);
                    delay(300);
                    clawServo.write(claw_close_angle);
                    delay(300);
                    BombDetection::bombEncounteredFlag = true;
                }

            } else {
                // don't pick up - bomb found
                BombDetection::bombEncounteredFlag = true;
                bombNowDetected = true;
            }
        } else {
            armServo.write(arm_lowered_angle);    
            delay(400);
            clawServo.write(claw_close_angle);
            delay(400);
        }

        armServo.write(arm_lifted_angle);
        
        if (!bombNowDetected || BombDetection::bombEncounteredFlag) {
            // lowered to lifted - no bomb
            delay(600);
        } else {
            // part lifted (below IR beacon) to lifted - bomb detected
            delay(100);
        }
        if (!bombNowDetected) {
            clawServo.write(claw_full_open_angle);
            delay(600);
        }
    }

    void collectTreasureUsingInterrupt() {
        BombDetection::configMagneticSensorPin(true);
        // should be in lifted arm, partly opened claw position

        // open claw as we lower arm
        clawServo.write(claw_part_open_angle);
        armServo.write(arm_part_lifted_angle);
        delay(400);
        clawServo.write(claw_full_open_angle);
        delay(250);

        // hall effect routine
        if (!BombDetection::bombEncounteredFlag) {
            armServo.write(arm_bomb_detect_angle);
            delay(300);
            if (!BombDetection::bombEncounteredFlag) {
                clawServo.write(claw_bomb_detect_angle);
                delay(400);
            } 
            if (!BombDetection::bombEncounteredFlag) {
                clawServo.write(claw_full_open_angle);
                delay(300);
                armServo.write(arm_lowered_angle);
                delay(600);
            }
            // check again for bomb - part so it doesn't knock off
            if (!BombDetection::bombEncounteredFlag) {
                clawServo.write(claw_part_open_angle);
                delay(300);
            }
            if (!BombDetection::bombEncounteredFlag) {
                clawServo.write(claw_close_angle);
                delay(400);
            }

            // bomb found along the routine
            if (BombDetection::bombEncounteredFlag) {
                clawServo.write(claw_part_open_angle);
                delay(300);
            } 
        } else {
            // already encountered bomb before - straight to treasure pickup
            armServo.write(arm_lowered_angle);    
            delay(400);
            clawServo.write(claw_close_angle);
            delay(400);
        }
        armServo.write(arm_lifted_angle);
        delay(500);
        clawServo.write(claw_full_open_angle);
        delay(300);
    }

    void deployBox() {
        boxServo.write(box_open_angle);
    }

    void deployBridge() {
        bridgeServo.write(box_open_angle);
    }
    void setServoPos(Servo servoObj, int deg) {
        servoObj.write(deg);
    }
}