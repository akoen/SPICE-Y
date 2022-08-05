#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
#include "ir.h"

namespace Executor {
    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(50, false, 16);
        // find tape - look for right first
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true);
        // follow tape & obtain second treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(2);
        // back up a bit
        // Encoders::driveMotorsDistance(50, false, 38);
        Encoders::driveMotorsDistance(50, false, 5);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 20, 3.5);
        TapeFollow::findBlackTape(40, Motors::min_rotate_dutyCycle, Motors::BOTH_WHEELS, false);
        Encoders::driveMotorsDistance(LW_PWM_DUTY, false, 5, 3.5);

        // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 40, 3.5);
        // Encoders::driveMotorsDistance(LW_PWM_DUTY, false, 10, 3.5);
        // TapeFollow::findBlackTape(40, Motors::min_rotate_dutyCycle, Motors::BOTH_WHEELS, false, 5);

        // // move fwd a bit
        // if (!Encoders::driveMotorsDistance(40, true, 5, 2.5)) {
        //     // timed out
        //     Encoders::driveMotorsDistance(40, false, 2);
        // }
        
        // // rotate some degs CCW about right wheel
        // int degs = 30;
        // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BACKWARDS, degs);
        
        // move back a bit
        // Encoders::driveMotorsDistance(40, false, 38);

        // find tape - look for left first
        ///////
        
        int whiteSurfaceCount = 0;
        int throughArchWhiteCount = 3000;

        while (true) {
            TapeFollow::driveWithPid();         
            // through archway
            if ((TapeFollow::onTapeL && TapeFollow::onTapeM && TapeFollow::onTapeR) || whiteSurfaceCount > throughArchWhiteCount) {            
                break;
            }
            if (!TapeFollow::onTapeL && !TapeFollow::onTapeM && !TapeFollow::onTapeR) {
                whiteSurfaceCount++;
            } else {
                whiteSurfaceCount = 0;
            }
        }
        // fit through archway
        int stopDuty = Motors::dutyCycleL > Motors::dutyCycleR ? Motors::dutyCycleL : Motors::dutyCycleR;
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, stopDuty, 50);
        // drive fwd some cm
        Encoders::driveMotorsDistance(LW_PWM_DUTY, true, 30);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 30, 2);
        // obtain third treasure using IR PID
        IR::driveWithPID();
        // obtain fourth treasure using IR PID

        // IR PID until robot hits beacon

        // turn right 90 degs and drive until edge detected

        // back up some cm and deploy bridge

        // drive backwards to bridge using edge detection PID or black tape PID

        // obtain treasure 

        // get into location and drop box

        // orient robot and drive up bridge

        // obtain gold treasure

    }
}