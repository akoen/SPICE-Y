#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
#include "ir.h"

namespace Executor {
    void archWayHandler(int dutyCycle, int timeout, double driveDist, Motors::RotateMode rotateMode, int dutyOffsetRW) {
        if (dutyCycle < Motors::min_drive_dutyCycle) dutyCycle = Motors::min_drive_dutyCycle;

        int delayMiliis = 10;
        int brakeDurationMillis = 50;

        while (!Encoders::driveMotorsDistance(dutyCycle, true, driveDist, timeout, dutyOffsetRW)) {
            // back up
            Encoders::driveMotorsDistance(dutyCycle, false, 25);
            Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::FORWARDS, 3);
        }
        // crossed arch - now readjust straight to IR
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BACKWARDS, 15);

        // drive fwd until fully out of arch
        Encoders::driveMotorsDistance(dutyCycle, true, 10);
    }

    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1, true);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(LW_PWM_DUTY, false, 7);
        // find tape - look for right first
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true);
        
        // follow tape & obtain second treasure - but don't return to original position
        TreasureDetection::obtainTapeTreasure(2, false);

        // get ready for archway
        Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 18);

        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 20, 2);
        Encoders::driveMotorsDistance(40, true, 15, 2);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 40, 3);
        // Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 60, 3);

        // go through archway
        int turnDuty = 20, offsetDutyRW = 50, timeout = 2;
        double driveDist = 40;
        archWayHandler(turnDuty, timeout, driveDist, Motors::RotateMode::FORWARDS, offsetDutyRW);
        
        // obtain third treasure using IR PID
        double driveFwdCm = 45;
        double rotateLeftDegs = 100;
        int driveDuty = 40;
        bool cacheThirdTreasure = false;
        TreasureDetection::obtainThirdIRtreasure(driveFwdCm, rotateLeftDegs, driveDuty, cacheThirdTreasure);

        // line up with IR beacon to start IR pid for fourth treasure
        Encoders::driveMotorsDistance(driveDuty, false, 18, 3);
        Encoders::rotateMotorsDegs(driveDuty, true, Motors::RotateMode::FORWARDS, 105, 3);

        // obtain fourth treasure using IR PID
        TreasureDetection::obtainIRTreasure(4);

        // back up & face beacon
        Encoders::driveMotorsDistance(driveDuty, false, 20, 3);
        // turn. If timeout, turned too far so turn back a bit
        // TODO: turn using IR vals
        if (!Encoders::rotateMotorsDegs(driveDuty, false, Motors::RotateMode::BOTH_WHEELS, 50, 3)) {
            Encoders::rotateMotorsDegs(driveDuty, true, Motors::RotateMode::BOTH_WHEELS, 25);
        }

        // IR PID until robot hits beacon
        long startTime = millis();
        long currTime = startTime;
        int timeoutPIDtoBeacon = 5;
        while (currTime > startTime + timeoutPIDtoBeacon * 1000) {
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

