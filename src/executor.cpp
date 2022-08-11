#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
#include "ir.h"
#include "servo-controller.h"

namespace Executor {
    void archWayHandler(int dutyCycle, int timeout, double driveDist, Motors::RotateMode rotateMode, int dutyOffsetRW) {
        if (dutyCycle < Motors::min_drive_dutyCycle) dutyCycle = Motors::min_drive_dutyCycle;

        while (!Encoders::driveMotorsDistance(dutyCycle, true, driveDist, timeout, dutyOffsetRW)) {
            // back up
            Encoders::driveMotorsDistance(dutyCycle, false, 20);
            Encoders::rotateMotorsDegs(40, false, Motors::RotateMode::FORWARDS, 5);
        }
        // crossed arch - now readjust straight to IR
        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BACKWARDS, 19, 2);
    }

    void secondTreasureToArchSetup() {
        // get ready for archway
        Encoders::driveMotorsDistance(55, false, 19);

        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 20, 2);
        Encoders::driveMotorsDistance(55, true, 17.3, 2);
        Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 33, 2);
    }

    void fourthTreasureToBeacon() { 
        int driveDuty = 60;
        int rotateDuty = 50;
        // back up a bit
        Encoders::driveMotorsDistance(50, false, 7.5, 1);
        // Encoders::rotateMotorsDegs(35, false, Motors::RotateMode::BOTH_WHEELS, 70);
        
        // multiple turns to ensure robot in line w/ beacon
        double firstTurnDegs = 45;
        double secondTurnDegs = 25;
        Encoders::rotateMotorsDegs(rotateDuty, false, Motors::RotateMode::BOTH_WHEELS, firstTurnDegs);
        Encoders::driveMotorsDistance(driveDuty, true, 16, 1);
        Encoders::rotateMotorsDegs(rotateDuty, false, Motors::RotateMode::BACKWARDS, secondTurnDegs);
        Encoders::driveMotorsDistance(driveDuty, false, 14, 1);

        // find IR
        // IR::findIR(30, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, true, 70, 30, 3);
        
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
    }

    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TapeFollow::checkChickenWire = false;
        TreasureDetection::obtainTapeTreasure(1, true);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(60, false, 8);
        // find tape - look for right first
        double searchAngle = 30;
        TapeFollow::findBlackTape(searchAngle, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, false);
        TapeFollow::checkChickenWire = true;
        // follow tape & obtain second treasure - but don't return to original position
        // TapeFollow::crossedChickenWire = true;
        TreasureDetection::obtainTapeTreasure(2, false);

        secondTreasureToArchSetup();
    
        // go through archway
        int turnDuty = 20, offsetDutyRW = 40, timeout = 3;
        double driveDist = 39;
        archWayHandler(turnDuty, timeout, driveDist, Motors::RotateMode::FORWARDS, offsetDutyRW);
        // drive fwd until fully out of arch

        int driveDuty = 70;
        
        Encoders::driveMotorsDistance(driveDuty, true, 11);
        // back up until front reflectance sensors see 1 1 1 or 0 1 0 (in case missed)
        Motors::driveBack(driveDuty);
        while (!(ReflectanceSensors::frontSensorLval && ReflectanceSensors::frontSensorMval && ReflectanceSensors::frontSensorRval) || !ReflectanceSensors::frontSensorMval) {
            ReflectanceSensors::readFrontReflectanceSensors();
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, driveDuty, 75);

        // obtain third treasure using IR PID
        double driveFwdCm = 5;
        double rotateLeftDegs = 87.1;
        bool cacheThirdTreasure = false;
        TreasureDetection::obtainThirdIRtreasure(driveFwdCm, rotateLeftDegs, driveDuty, cacheThirdTreasure);

        // line up with IR beacon to start IR pid for fourth treasure
        Encoders::driveMotorsDistance(driveDuty, false, 16, 3);
        Encoders::rotateMotorsDegs(driveDuty, true, Motors::RotateMode::BOTH_WHEELS, 85, 2);

        // decrease max duty cycle so more/more reliable sonar readings can be taken (serial this to be sure)
        Motors::max_drive_dutyCycle = 40;

        // obtain fourth treasure using IR PID and return to original pos
        TreasureDetection::obtainIRTreasure(4, false);
    
        Motors::max_drive_dutyCycle = 85;
        
        fourthTreasureToBeacon();
        
        // back up a bit and turn right about 90 degs and drive until edge detected
        double firstTurnDeg = 30;
        double secondTurnDeg = 56.5;
        int rotateDuty = 25;
        Encoders::driveMotorsDistance(driveDuty, false, 6, 1.5);
        Encoders::rotateMotorsDegs(rotateDuty, false, Motors::RotateMode::BOTH_WHEELS, firstTurnDeg, 1.5);
        Encoders::driveMotorsDistance(driveDuty, true, 12);
        Encoders::rotateMotorsDegs(rotateDuty, false, Motors::RotateMode::BOTH_WHEELS, secondTurnDeg, 1.5);
        Encoders::driveMotorsDistance(driveDuty, false, 36);

        // /*
        // // hit pole
        // Encoders::driveMotorsDistance(driveDuty, true, 25, 2);
        // // drive back
        // Encoders::driveMotorsDistance(driveDuty, false, 50, 3);
        // */
        
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle, 30, 50);
        delay(10);
        ReflectanceSensors::readSideReflectanceSensors();
        // left on white, right is not
        if (!ReflectanceSensors::sideSensorLval) {
            Motors::rotate(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::BACKWARDS);
            while (!ReflectanceSensors::sideSensorLval) {
                ReflectanceSensors::readSideReflectanceSensors();
                Serial.print("Rotating left: ");
                Serial.print(ReflectanceSensors::sideSensorLval);
                Serial.print(" ");
                Serial.println(ReflectanceSensors::sideSensorRval);
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_LEFT, Motors::RotateMode::BACKWARDS, 30, 50);
        } else if (!ReflectanceSensors::sideSensorRval) {
            Motors::rotate(Motors::min_rotate_dutyCycle, true, Motors::RotateMode::BACKWARDS);
            while (!ReflectanceSensors::sideSensorRval) {
                ReflectanceSensors::readSideReflectanceSensors();
                Serial.print("Rotating right: ");
                Serial.print(ReflectanceSensors::sideSensorLval);
                Serial.print(" ");
                Serial.println(ReflectanceSensors::sideSensorRval);
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, Motors::RotateMode::BACKWARDS, 30, 50);
        }       
        // drive fwd - less than expected due to the high duty cycle brake when using relfectance sensors
        Encoders::driveMotorsDistance(50, true, 4.3, 1);
        delay(80);
        // deploy bridge
        Servos::deployBridge();
        delay(500);
        Servos::bridgeServo.write(Servos::bridge_closed_angle);
        Encoders::driveMotorsDistance(30, true, 3.5, 1);
        delay(80);
        // drive back over bridge completely
        Encoders::driveMotorsDistance(75, false, 80);
        
        // hit tape on bridge for ref
        Motors::driveFwd(35);
        ReflectanceSensors::readFrontReflectanceSensors();
        while (!ReflectanceSensors::frontSensorLval && !ReflectanceSensors::frontSensorMval && !ReflectanceSensors::frontSensorRval) {
            ReflectanceSensors::readFrontReflectanceSensors();
            Serial.print("Reflectance: ");
            Serial.print(ReflectanceSensors::frontSensorLval);
            Serial.print(" ");
            Serial.println(ReflectanceSensors::frontSensorMval);
            Serial.print(" ");
            Serial.println(ReflectanceSensors::frontSensorRval);
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, 50, 60);
        delay(80);

        TreasureDetection::obtainFifthTreasure(45, 39, 21, false);
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

