#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
#include "ir.h"
#include "servo-controller.h"

namespace Executor {
    void archWayHandler(int dutyCycle, int timeout, double driveDist, Motors::RotateMode rotateMode, int dutyOffsetRW) {
        if (dutyCycle < Motors::min_drive_dutyCycle) dutyCycle = Motors::min_drive_dutyCycle;

        int delayMiliis = 10;
        int brakeDurationMillis = 50;

        while (!Encoders::driveMotorsDistance(dutyCycle, true, driveDist, timeout, dutyOffsetRW)) {
            // back up
            Encoders::driveMotorsDistance(dutyCycle, false, 25);
            Encoders::rotateMotorsDegs(40, false, Motors::RotateMode::FORWARDS, 3);
        }
        // crossed arch - now readjust straight to IR
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BACKWARDS, 10);
    }

    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1, true);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(50, false, 7);
        // find tape - look for right first
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true);
        
        // follow tape & obtain second treasure - but don't return to original position
        // TapeFollow::crossedChickenWire = true;
        TreasureDetection::obtainTapeTreasure(2, false);

        // get ready for archway
        Encoders::driveMotorsDistance(40, false, 18);

        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 20, 2);
        Encoders::driveMotorsDistance(40, true, 16.5, 2);
        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 35, 2);

        // go through archway
        int turnDuty = 20, offsetDutyRW = 40, timeout = 2;
        double driveDist = 42;
        archWayHandler(turnDuty, timeout, driveDist, Motors::RotateMode::FORWARDS, offsetDutyRW);
        
        // drive fwd until fully out of arch
        int driveDuty = 40;

        Encoders::driveMotorsDistance(driveDuty, true, 14);
        // back up until front reflectance sensors see 1 1 1 or 0 1 0 (in case missed)
        Motors::driveBack(driveDuty);
        while (!(ReflectanceSensors::frontSensorLval && ReflectanceSensors::frontSensorMval && ReflectanceSensors::frontSensorRval) || !ReflectanceSensors::frontSensorMval) {
            ReflectanceSensors::readFrontReflectanceSensors();
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, driveDuty, 50);

        // obtain third treasure using IR PID
        double driveFwdCm = 6;
        double rotateLeftDegs = 100;
        bool cacheThirdTreasure = false;
        TreasureDetection::obtainThirdIRtreasure(driveFwdCm, rotateLeftDegs, driveDuty, cacheThirdTreasure);

        // line up with IR beacon to start IR pid for fourth treasure
        Encoders::driveMotorsDistance(driveDuty, false, 16.5, 3);
        Encoders::rotateMotorsDegs(driveDuty, true, Motors::RotateMode::BOTH_WHEELS, 90, 2);

        // decrease max duty cycle so more/more reliable sonar readings can be taken (serial this to be sure)
        Motors::max_drive_dutyCycle = 37;

        // obtain fourth treasure using IR PID and return to original pos
        TreasureDetection::obtainIRTreasure(4, false);
        
        Motors::max_drive_dutyCycle = 80;

        // back up a bit
        Encoders::driveMotorsDistance(driveDuty, false, 9, 1);
        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 58);
        // find IR
        // IR::findIR(30, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true, 70, 30, 3);

        ////
        // back up & face beacon
        // Encoders::driveMotorsDistance(driveDuty, false, 20, 3);
        // // turn. If timeout, turned too far so turn back a bit
        // // TODO: turn using IR vals
        // if (!Encoders::rotateMotorsDegs(driveDuty, false, Motors::RotateMode::BOTH_WHEELS, 40, 3)) {
        //     Encoders::rotateMotorsDegs(driveDuty, true, Motors::RotateMode::BOTH_WHEELS, 25);
        // }
        ////
        
        // IR PID until robot hits beacon
        long startTime = millis();
        long currTime = startTime;
        int timeoutPIDtoBeacon = 3;
        int timeoutDrive = 1.5;
        // TODO: or sonar sees top of V dist
        while (currTime < startTime + timeoutPIDtoBeacon * 1000) {
            IR::driveWithPID();
            Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
            currTime = millis();
        }
        // Encoders::driveMotorsDistance(30, false, 5, 1);
        // Encoders::driveMotorsDistance(30, true, 7, 1, 40);

        // back up a bit and turn right about 90 degs and drive until edge detected
        double firstTurnDeg = 60;
        double secondTurnDeg = 40;

        Encoders::driveMotorsDistance(driveDuty, false, 5);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, firstTurnDeg, 1.5);
        Encoders::driveMotorsDistance(driveDuty, true, 10);
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, secondTurnDeg, 1.5);
        
        // Encoders::driveMotorsDistance(driveDuty, false, 30, 2);

        Motors::driveBack(Motors::min_drive_dutyCycle);
        // both reflectance sensors on surface
        while (!ReflectanceSensors::sideSensorLval || !ReflectanceSensors::sideSensorRval) {
            ReflectanceSensors::readSideReflectanceSensors();
        }
        // stop w/ greater PWM in case wheels are falling off
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, 30, 100);
 
        // left on white, right is not
        if (!ReflectanceSensors::sideSensorLval) {
            Motors::rotate(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::BOTH_WHEELS);
            while (!ReflectanceSensors::sideSensorLval) {
                ReflectanceSensors::readSideReflectanceSensors();
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_LEFT, Motors::RotateMode::BOTH_WHEELS, Motors::min_rotate_dutyCycle, 50);
        } else if (!ReflectanceSensors::sideSensorRval) {
            Motors::rotate(Motors::min_rotate_dutyCycle, true, Motors::RotateMode::BOTH_WHEELS);
            while (!ReflectanceSensors::sideSensorLval) {
                ReflectanceSensors::readSideReflectanceSensors();
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, Motors::RotateMode::BOTH_WHEELS, Motors::min_rotate_dutyCycle, 50);
        }

        // drive fwd
        Encoders::driveMotorsDistance(70, true, 3.5, 1);
        delay(600);
        // deploy bridge
        Servos::deployBridge();
        delay(600);
        Encoders::driveMotorsDistance(30, true, 5, 1);
        delay(500);
        // drive back
        Encoders::driveMotorsDistance(80, false, 65);

        TreasureDetection::obtainFourthTreasure(30, 40, 14, false);
        while (true) {
            Serial.print(Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT));
            Serial.print(" ");
            Serial.println(Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT));
        }

        // drive backwards to bridge using edge detection PID or black tape PID

        // obtain treasure 

        // get into location and drop box

        // orient robot and drive up bridge

        // obtain gold treasure

    }
}

