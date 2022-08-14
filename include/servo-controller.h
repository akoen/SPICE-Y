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
    extern const int arm_part_lifted_angle;
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
     * Claw routine for picking up and dropping treasures into storage. Handles bomb detection.
     * Assumes arm and claw are in lifted and closed claw positions, respectively, at the start. Routine ends with the 
     * arm lifted and claw opened. 
     * Claw should be closed externally afterwards when appropriate. This choice was due to minimize delays.
     */
    void collectTreasure();

    /**
     * Collects the treasures and handles bomb detection using interrupts
     */
    void collectTreasureUsingInterrupt();

    /**
     * Deploys the storage box.
     */
    void deployBox();
    
    /**
     * Deploys the bridge
     */
    void deployBridge();

    /**
     * Sets the specified servo to the specifed angle (deg).
     */
    void setServoPos(Servo servoObj, int deg);    
}

#endif