#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"

namespace TreasureDetection {
    // 6 potential treasures that needs to be located 
    const double side_sonar_treasure_dists[6] = {25, 32, 20, 20, 20, 20}; // cm
    const double side_sonar_treasure_dists_err[6] = {10, 8, 10, 2.5, 10, 10}; // cm

    const double front_sonar_treasure_dists[6] = {22, 28, 12.5, 15, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {20, 8, 10, 5}; // cm 

    const double treasure_in_claw_dist = 14.5; // cm
    const double treasure_in_claw_dist_err = 0.5; // cm

    const double def_drive_to_treasure_duty = Motors::min_drive_dutyCycle; // %

    const int side_sonar_req_good_readings[6] = {2, 3, 3, 3, 3, 3};
    const int front_sonar_req_good_readings[6] = {3, 3, 1, 1, 1, 1};
    const int claw_req_good_readings[6] = {5, 5, 3, 3, 3, 3};

    const double near_treasure_dists[6] = {120, 34};


    bool obtainTapeTreasure(int treasureNum) {

        Servos::configArmClawPins();

        double firstSideSonarTreausureDist = side_sonar_treasure_dists[treasureNum-1];
        double firstSideSonarTreausureDistErr = side_sonar_treasure_dists_err[treasureNum-1];
        
        double firstFrontSonarTreausureDist = front_sonar_treasure_dists[treasureNum-1];
        double firstFrontSonarTreausureDistErr = front_sonar_treasure_dists_err[treasureNum-1];

        // tape follow using PID until treasure located
        double loopCount = 0;
        double rightSonarDist = 0;
        int goodSideSonarReadings = 0;

        // only tape follow & not look at sonar when not near treasure
        // TapeFollow::driveWithPidDist(near_treasure_dists[treasureNum-1]);
        // look for tape
        // TapeFollow::findBlackTape(20, Motors::min_rotate_dutyCycle, Motors::FORWARDS);

        // look for treasure since now near
        while (goodSideSonarReadings < side_sonar_req_good_readings[treasureNum - 1]) {
            TapeFollow::driveWithPid();
            // look for second treasure when passed chicken wire
            if (treasureNum == 2 && !TapeFollow::crossedChickenWire) rightSonarDist = -1;
            else rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_R);

            Serial.println("Right sonar dist: ");
            Serial.println(rightSonarDist);

            ReflectanceSensors::printFrontReflectance();

            // consecutive good readings
            if (rightSonarDist < firstSideSonarTreausureDist + firstSideSonarTreausureDistErr && rightSonarDist > firstSideSonarTreausureDist - firstSideSonarTreausureDistErr) {
                goodSideSonarReadings++;
            } else {
                goodSideSonarReadings = 0;
            }
            // if inf loop --> return false (some sort of timeout)
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, LW_PWM_DUTY, 50);
        Serial.println("found treasure right: ");    
        Serial.println(rightSonarDist);    

        // rotate until treasure not seen for second treasure
        if (treasureNum == 2) {
            Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, true, Motors::RotateMode::BACKWARDS, 70);
        }

        Sonars::SonarType sonarType;
        // which sonar to check
        if (treasureNum == 2) {
            sonarType = Sonars::SonarType::LEFT;
        } else {
            sonarType = Sonars::SonarType::RIGHT;
        }
        return treasureCollectionRoutine(sonarType, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, true, treasureNum);
    }

    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum) {
        // drive forward a bit if needed
        int fwdDriveCalibration = 0;
        if (treasureNum == 1) {
                fwdDriveCalibration = 7;    // cm - so wheel not caught on ramp
                Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, true, fwdDriveCalibration);
        }
        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {
            Motors::MotorAction action = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT; 
            
            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            Motors::RotateMode treasureTurnRotateMode = treasureNum == 2 ? Motors::RotateMode::FORWARDS : Motors::RotateMode::BACKWARDS;
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm);

            Serial.println("Front sonar: ");
            // delay(1000);

            double distFrontSonar;
            int goodReadingsCount = 0;
            Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);

            // turn until certain within threshold
            long pulsesStart, pulsesEnd;
            while (goodReadingsCount < front_sonar_req_good_readings[treasureNum-1]) {
                distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
                if (distFrontSonar < distFront + distFrontErr && distFrontSonar > distFront - distFrontErr) {
                    goodReadingsCount++;
                }
                else {
                    goodReadingsCount = 0;
                }
                Serial.println(distFrontSonar);
            }

            // while (true) {
            //     distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            //     if (distFrontSonar < distFront + distFrontErr && distFrontSonar > distFront - distFrontErr) {
            //         // first good reading
            //         if (goodReadingsCount == 0) {
            //             switch(action) {
            //                 case Motors::MotorAction::ROTATE_RIGHT:
            //                     pulsesStart = treasureTurnRotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
            //                 case Motors::MotorAction::ROTATE_LEFT:
            //                     pulsesStart = treasureTurnRotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseLW : Encoders::pulseRW;
            //             }
            //             // Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::min_rotate_dutyCycle, 50);
            //             // Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            //         }
            //         goodReadingsCount++;
            //     }
            //     else {
            //         // last good reading
            //         if (goodReadingsCount > front_sonar_req_good_readings[treasureNum-1]) {
            //             // Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::min_rotate_dutyCycle, 50);
            //             switch(action) {
            //                 case Motors::MotorAction::ROTATE_RIGHT:
            //                     pulsesEnd = treasureTurnRotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
            //                 case Motors::MotorAction::ROTATE_LEFT:
            //                     pulsesEnd = treasureTurnRotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseLW : Encoders::pulseRW;
            //             }
            //             break;
            //         }
            //         goodReadingsCount = 0;
            //     }
            //     Serial.println(distFrontSonar);
            // }
            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm, 200);
            // // go halfway of first & last good readings
            // int pulseInterval = abs(pulsesEnd - pulsesStart);
            // Serial.print("Pulse end ");
            // Serial.print(pulsesEnd);
            // Serial.print("Pulse start");
            // Serial.println(pulsesStart);
            // delay(2000);
            // Motors::MotorAction inverseAction;
            // Motors::RotateMode inverseRotate;
            // std::tie(inverseAction, inverseRotate) = Motors::getInverseDrive(action, treasureTurnRotateMode);
            // Encoders::driveMotorsEncoderPulses(Motors::default_rotate_pwm, inverseAction, inverseRotate, pulseInterval / 2);

            // empirical - 2nd treasure requires some degs to be rotated to the left to be aligned to treasure
            if (treasureNum == 2) {
                Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::FORWARDS, 6);
            }

            if (retOriginalPos) Encoders::endAddActionCache();
        }

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        Motors::setDir(true, true);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();
        
        // go to treasure until in claw range
        double distFrontSonarTreasureClaw;

        int goodReadingsCount = 0;

        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonarTreasureClaw);            
            distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

            if ()
            if (distFrontSonarTreasureClaw < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;
        }

        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::min_drive_dutyCycle, 100);
        
        // too close to the treasure
        Motors::setDir(false, false);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();
        
        goodReadingsCount = 0;
        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonarTreasureClaw);            
            distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

            if (distFrontSonarTreasureClaw > treasure_in_claw_dist - treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 100);

        if (retOriginalPos) Encoders::endAddActionCache();
    
        Serial.println("found treasure claw: ");    
        Serial.println(distFrontSonarTreasureClaw);   
        
        // collect
        Servos::collectTreasure();

        // return to original position
        if (retOriginalPos) Encoders::executeReverseCache();

        return true;
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */