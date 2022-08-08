#ifndef SERVO_CONTROLLER
#define SERVO_CONTROLLER

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

namespace Servos {
    extern const int claw_close_angle;  // deg
    extern const int claw_part_open_angle;  // so it doesn't hit box when lifted
    extern const int claw_full_open_angle;
    extern const int claw_bomb_detect_angle;
    
    extern const int arm_bomb_detect_angle;
    extern const int above_treasure_below_IR_angle;
    extern const int arm_lowered_angle;
    extern const int arm_lifted_angle;

    extern const int box_closed_angle;
    extern const int box_open_angle;
    
    extern const int bridge_closed_angle;
    extern const int bridge_open_angle;

    extern Servo clawServo;
    extern Servo armServo;
    extern Servo bridgeServo;
    extern Servo boxServo;

    extern bool clawPinsConfiged;
    extern bool armPinsConfiged;
    extern bool bridgePinsConfiged;
    extern bool boxPinsConfiged;
    /**
     * Configures all servo pins if they have yet been configured.
     */
    void configAllServoPins();
    
    /**
     * Configures claw and arm servo pins if they have yet been configured.
     */ 
    void configArmClawPins();

    /**
     * Claw picks up and drops treasure into storage. Handles bomb detection
     * Starting from arm lifted, claw in part open position and ends in
     * this position.
     */
    void collectTreasure();

    void collectTreasureUsingInterrupt();

    /**
     * Deploys the storage box to be released.
     */
    void deployBox();
    
    void deployBridge();

    void setServoPos(Servo servoObj, int deg);    
}

#endif