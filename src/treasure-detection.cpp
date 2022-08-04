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
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BACKWARDS, 70);
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
        double avgFoundTreasureFrontDists = 0;
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
                    avgFoundTreasureFrontDists += distFrontSonar;
                }
                else {
                    goodReadingsCount = 0;
                }
                Serial.println(distFrontSonar);
            }
            avgFoundTreasureFrontDists /= (1.0* front_sonar_req_good_readings[treasureNum-1]);

            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm, 200);

            // empirical - 2nd treasure requires some degs to be rotated to the left to be aligned to treasure
            if (treasureNum == 2) {
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 6);
            }

            if (retOriginalPos) Encoders::endAddActionCache();
        }

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        Motors::setDir(true, true);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();
        
        // go to treasure until in claw range
        Serial.print("avg found treasure front: ");
        Serial.println(avgFoundTreasureFrontDists);
        delay(1000);
        driveToTreasureFrontSonar(avgFoundTreasureFrontDists-10, 5);
        // driveToTreasureFrontSonar(front_sonar_treasure_dists[treasureNum-1], 5);

        // double distFrontSonarTreasureClaw;
        // int goodReadingsCount = 0;

        // /* Ensuring driving into claw range
        //  *  1) drive until claw in range. If distance gets larger then rotate left and right until expected distance found
        //  *  2) drive until very close to pedestal (< 2 cm) using encoders & obtained sonar values. If the sonar value read is expected (~2 cm) then claw is in range.
        //  *  Otherwise, back up some more cms and rotate some degs left & right until expected distance found (2+backed up cm)
        //  */

        // while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
        //     Serial.print("Front sonar dist: ");
        //     Serial.println(distFrontSonarTreasureClaw);            
        //     distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
        //     if (distFrontSonarTreasureClaw < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
        //     else goodReadingsCount = 0;
        // }

        // Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::min_drive_dutyCycle, 100);
        
        // // too close to the treasure
        // Motors::setDir(false, false);
        // Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        // Motors::drive();
        
        // goodReadingsCount = 0;
        // while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
        //     Serial.print("Front sonar dist: ");
        //     Serial.println(distFrontSonarTreasureClaw);            
        //     distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

        //     if (distFrontSonarTreasureClaw > treasure_in_claw_dist - treasure_in_claw_dist_err) goodReadingsCount++;
        //     else goodReadingsCount = 0;
        // }
        // Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 100);

        if (retOriginalPos) Encoders::endAddActionCache();
    
        // Serial.println("found treasure claw: ");    
        // Serial.println(distFrontSonarTreasureClaw);   
        
        // collect
        Servos::collectTreasure();

        // return to original position
        if (retOriginalPos) Encoders::executeReverseCache();

        return true;
    }
    void driveToTreasureFrontSonar(double treasureDist, int numReadings) {
        Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, true, treasureDist);
        // check sonar
        double sonarDist = 0;
        double expectedDistRange = 12;   // cm
        int numGoodReadings = 0;
        bool found = false;
        double avgSonarDist = 0;

        while (numGoodReadings < numReadings) {
            sonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            
            // 1 reading out of numReadings out of range, need to find & recalibrate into position by rotating 
            if (sonarDist > expectedDistRange) {
                // back up some cms
                int backUpDist = 4;
                Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, backUpDist);

                // check sonar values while rotating
                Motors::MotorAction rotateAction;
                Motors::RotateMode rotateMode = Motors::RotateMode::BOTH_WHEELS;

                int radius = rotateMode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;  // both wheels
                int rotateDegs = 30;
                int turnPulsesInterval = Encoders::degsToPulses(rotateDegs, radius);
                int rotateDuty = Motors::default_rotate_pwm;

                long startPulses;
                long checkPulses;
                for (int i = 0; i < 2; i++) {
                    if (i == 0) {
                        // rotate left both wheels
                        startPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseLW : Encoders::pulseRW;
                        checkPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseLW : Encoders::pulseRW;
                    } else {
                        // rotate right both wheels
                        startPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
                        checkPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
                    }
                    Motors::rotate(rotateDuty, false, rotateMode);
                    int rotateNumReadings = 0;
                    while (true) {
                        sonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
                        if (sonarDist < expectedDistRange) {
                            avgSonarDist += sonarDist;
                            rotateNumReadings++;
                        }
                        // loop end conditions
                        // found
                        if (rotateNumReadings == 1) {
                        // if (rotateNumReadings == numGoodReadings) {
                            // avgSonarDist /= (1.0 * numGoodReadings);
                            avgSonarDist /= (1.0 * 1);
                            found = true;
                            break;
                        }
                        // not found(s)
                        if (rotateMode == Motors::RotateMode::BACKWARDS) {
                            if (checkPulses < startPulses - turnPulsesInterval) {
                                break;
                            }
                        } else {
                            if (checkPulses > startPulses + turnPulsesInterval) {
                                break;
                            }
                        }
                        // update
                        if (i == 0) {
                            // rotate left both wheels
                            checkPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseLW : Encoders::pulseRW;
                        } else {
                            // rotate right both wheels
                            checkPulses = rotateMode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
                        }
                    }
                    Motors::stopWithBrake(Motors::MotorAction::ROTATE_LEFT, Motors::RotateMode::BOTH_WHEELS, rotateDuty, 50);
                    // found 
                    if (found) {
                        // back up to claw range
                        double distToClawRange = TreasureDetection::treasure_in_claw_dist - (avgSonarDist + backUpDist);
                        bool driveFwd;
                        if (distToClawRange > 0) {
                            driveFwd = false;
                        } else {
                            driveFwd = true;
                            distToClawRange *= -1;
                        }
                        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, driveFwd, distToClawRange);

                        return;
                    } 
                    // not found - rotate back same degs and check again 
                    Encoders::rotateMotorsDegs(rotateDuty, rotateAction == Motors::MotorAction::ROTATE_RIGHT, rotateMode, rotateDegs);
                }
            } else {
                numGoodReadings++;
                avgSonarDist += sonarDist;
            }
        }
        avgSonarDist /= (1.0 * numGoodReadings);
        // out of loop & not returned, in front of pedestal already            
        double distToClawRange = TreasureDetection::treasure_in_claw_dist - (avgSonarDist);
        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, distToClawRange);
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */