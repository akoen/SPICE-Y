#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
#include "ir.h"

namespace Executor {
    void archWayHandler(int dutyCycle, int timeout, int driveDist, Motors::RotateMode rotateMode, int dutyOffsetRW) {
        // note: the lower of the pulses (left)

        if (dutyCycle < Motors::min_drive_dutyCycle) dutyCycle = Motors::min_drive_dutyCycle;

        int delayMiliis = 10;
        int brakeDurationMillis = 50;

        // bool crossedArch = true;
        // while (true) {
        //     // assume true at first - will be false in same loop if not
        //     crossedArch = true;

        //     long startMillis = millis();
        //     long currMillis = startMillis;

        //     Motors::setDir(true, true);

        //     Motors::setDutyCycles(dutyCycle, dutyCycle + dutyOffsetRW);
        //     Motors::drive();

        //     long startPulseLW = Encoders::pulseLW;
        //     long startPulseRW = Encoders::pulseRW;
        //     int pulseInterval = Encoders::cmToPulses(driveDist);

        //     while (Encoders::pulseLW < startPulseLW + pulseInterval && Encoders::pulseRW < startPulseRW + pulseInterval) {                    
        //         currMillis = millis();
        //         if (currMillis > startMillis + timeout*1000) {
        //             crossedArch = false;
        //             break;
        //         }
        //     }
        //     delay(delayMiliis);
        //     Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, rotateMode, dutyCycle, brakeDurationMillis);
        //     // check if crossed arch
        //     if (!crossedArch) {
        //         // have not - back up to get ready to try again
        //         Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 25);
        //         Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::FORWARDS, 5);
        //     } else {
        //         break;
        //     }
        //     delay(delayMiliis);
        //     Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, dutyCycle, brakeDurationMillis);
        // }

        while (!Encoders::driveMotorsDistance(dutyCycle, true, driveDist, timeout, dutyOffsetRW)) {
            // back up
            Encoders::driveMotorsDistance(dutyCycle, false, 25);
            Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::FORWARDS, 5);
        }
        // crossed arch - now readjust straight to IR
        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BACKWARDS, 30);
    }

    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(LW_PWM_DUTY, false, 5);
        // find tape - look for right first
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true);
        // follow tape & obtain second treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(2);

        // get ready for archway
        Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 15);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 65, 3);

        // archway handler
        archWayHandler(TreasureDetection::def_drive_to_treasure_duty, 3, 30, Motors::RotateMode::FORWARDS, TreasureDetection::def_drive_to_treasure_duty);

        
        // obtain third treasure using IR PID
        // TreasureDetection::obtainIRTreasure(3);
        while (true) {
            IR::driveWithPID();
        }

        // obtain fourth treasure using IR PID
        TreasureDetection::obtainIRTreasure(4);
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

