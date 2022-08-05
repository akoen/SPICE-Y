#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
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
        Encoders::driveMotorsDistance(50, false, 38);

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
        TapeFollow::findBlackTape(50, Motors::min_rotate_dutyCycle, Motors::BOTH_WHEELS, false);
        
        int whiteSurfaceCount = 0;
        int throughArch = 20;

        while (true) {
            TapeFollow::driveWithPid();
            
            // // through archway
            // if ((TapeFollow::onTapeL && TapeFollow::onTapeM && TapeFollow::onTapeR) || whiteSurfaceCount > throughArch) {
            //     break;
            // }
            // if (!TapeFollow::onTapeL && !TapeFollow::onTapeM && !TapeFollow::onTapeR) {
            //     whiteSurfaceCount++;
            // } else {
            //     whiteSurfaceCount = 0;
            // }
        }
        int stopDuty = Motors::dutyCycleL > Motors::dutyCycleR ? Motors::dutyCycleL : Motors::dutyCycleR;
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, stopDuty, 50);
        // drive fwd 20 cm
        Encoders::driveMotorsDistance(LW_PWM_DUTY, true, 20);
        

        // fit through archway

        // obtain third treasure using IR PID

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