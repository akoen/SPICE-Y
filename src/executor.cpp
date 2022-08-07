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

        while (!Encoders::driveMotorsDistance(dutyCycle, true, driveDist, timeout, dutyOffsetRW)) {
            // back up
            Encoders::driveMotorsDistance(dutyCycle, false, 25);
            Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::FORWARDS, 3);
        }
        // 
        // crossed arch - now readjust straight to IR
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BACKWARDS, 15);
    }

    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1, true);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(LW_PWM_DUTY, false, 5);
        // find tape - look for right first
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true);
        
        // follow tape & obtain second treasure
        TreasureDetection::obtainTapeTreasure(2, false);

        // get ready for archway
        Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 18);

        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 20, 2);
        Encoders::driveMotorsDistance(40, true, 15, 2);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 40, 3);
        // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 60, 3);

        // archway handler
        archWayHandler(20, 2, 40, Motors::RotateMode::FORWARDS, 50);
        // obtain third treasure using IR PID
        TreasureDetection::obtainIRTreasure2(3, false);

        // drive back 
        Encoders::driveMotorsDistance(40, false, 18, 3);
        Encoders::rotateMotorsDegs(40, true, Motors::RotateMode::FORWARDS, 105, 3);

        // obtain fourth treasure using IR PID
        TreasureDetection::obtainIRTreasure(4);

        // back up & face beacon
        Encoders::driveMotorsDistance(40, false, 20, 3);
        // turn. If timeout, turned too far so turn back a bit
        // TODO: turn using IR valsd
        if (!Encoders::rotateMotorsDegs(40, false, Motors::RotateMode::BOTH_WHEELS, 50, 3)) {
            Encoders::rotateMotorsDegs(40, true, Motors::RotateMode::BOTH_WHEELS, 25);
        }

        // IR PID until robot hits beacon
        long startTime = millis();
        long currTime = startTime;
        int timeout = 5;
        while (currTime > startTime + timeout * 1000) {
            IR::driveWithPID();
            Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
        }


        // turn right 90 degs and drive until edge detected

        // back up some cm and deploy bridge

        // drive backwards to bridge using edge detection PID or black tape PID

        // obtain treasure 

        // get into location and drop box

        // orient robot and drive up bridge

        // obtain gold treasure

    }
}

