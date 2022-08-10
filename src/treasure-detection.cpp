#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"
#include "ir.h"
#include <cmath>

namespace TreasureDetection {
    /* For sonar */
    // 6 potential treasures that needs to be located 
    const double side_sonar_treasure_dists[6] = {25, 29, 40, 17, 20, 20}; // cm
    const double side_sonar_treasure_dists_err[6] = {15 , 7, 7, 13, 10, 10}; // cm

    const double front_sonar_treasure_dists[6] = {24, 30, 24, 17, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {11, 12, 10, 12, 10, 7}; // cm 

    const double treasure_in_claw_dist = 14.5; // cm
    const double treasure_in_claw_dist_err = 1.5; // cm

    // extern const double treasure_in_v_dist = 12;
    extern const double treasure_in_v_dist = 15;

    const int side_sonar_req_good_readings[6] = {2, 2, 2, 3, 2, 2};
    const int front_sonar_req_good_readings[6] = {2, 2, 2, 2, 2, 2};
    const int claw_req_good_readings[6] = {2, 2, 2, 2, 2, 2};

    /* For encoder */
    const double v_to_claw_dist = 5.9;
    const double near_treasure_dists[6] = {120, 34};

    /* other */
    const double def_drive_to_treasure_duty = 30; // %


    bool obtainTapeTreasure(int treasureNum, bool retToOriginalPos) {
        Servos::configArmClawPins();

        double firstSideSonarTreausureDist = side_sonar_treasure_dists[treasureNum-1];
        double firstSideSonarTreausureDistErr = side_sonar_treasure_dists_err[treasureNum-1];
        
        double firstFrontSonarTreausureDist = front_sonar_treasure_dists[treasureNum-1];
        double firstFrontSonarTreausureDistErr = front_sonar_treasure_dists_err[treasureNum-1];

        // tape follow using PID until treasure located
        double loopCount = 0;
        double rightSonarDist = 0;
        int goodSideSonarReadings = 0;

        long startMillis = millis();
        long currMillis = startMillis;

        while (goodSideSonarReadings < side_sonar_req_good_readings[treasureNum - 1]) {
            TapeFollow::driveWithPid();

            // only look for second treasure when passed chicken wire
            if (treasureNum == 2 && !TapeFollow::crossedChickenWire) rightSonarDist = -1;
            else {
                // sonar every 35 ms
                if (currMillis > startMillis + 35) {
                    rightSonarDist = Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT, 0);
                    startMillis = currMillis;
                }
            }
            currMillis = millis();
            
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
            // if (treasureNum == 1 && TapeFollow::crossedChickenWire) {
            //     return false;
            // }
        }
        // Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, LW_PWM_DUTY, 50);
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::dutyCycleL + 30, 50, Motors::default_motors_stop_millis, Motors::dutyCycleR - Motors::dutyCycleL + 30);

        Serial.print("Found treasure right: ");    
        Serial.println(rightSonarDist);    

        // 2nd treasure special: rotate until treasure not seen for second treasure
        if (treasureNum == 2) {
            double driveCalibration = 5;
            if (retToOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_BACK, Motors::NONE, Motors::min_drive_dutyCycle);
            // drive back a bit
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, driveCalibration);
            if (retToOriginalPos) Encoders::endAddActionCache();
 
            // rotate right past treasure
            if (retToOriginalPos) Encoders::startAddActionCache(Motors::ROTATE_RIGHT, Motors::BACKWARDS, Motors::default_rotate_pwm);
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BACKWARDS, 65);
            if (retToOriginalPos) Encoders::endAddActionCache();
        }

        Sonars::SonarType sonarType = Sonars::SonarType::RIGHT;

        return treasureCollectionRoutine(sonarType, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, retToOriginalPos, treasureNum);
    }

    void driveBackToTreasureFrontSonar(int);

    bool obtainThirdIRtreasure(double driveFwd, double rotateLeftDegs, int driveDuty, bool cache) {
        int timeout = 3;
    
        // drive fwd a bit to get ready for IR PID
        Encoders::driveMotorsDistance(driveDuty, true, 40, 2);
        /*
        // IR PID for 1.5 secs
        long startMillis = millis();
        long currMillis = startMillis;
        while (currMillis < startMillis + 1.5*1000) {
            IR::driveWithPID();
            currMillis = millis();
        }
        */

        long startEncoderLW = Encoders::pulseLW;
        long startEncoderRW = Encoders::pulseRW;
        long checkEncoderLW = startEncoderLW;
        long checkEncoderRW = startEncoderRW;

        double IRdriveDist = 27;
        int IRpulsesInterval = Encoders::cmToPulses(IRdriveDist);
        while (checkEncoderLW < startEncoderLW + IRpulsesInterval && checkEncoderRW < startEncoderRW + IRpulsesInterval) {
            IR::driveWithPID();
            checkEncoderLW = Encoders::pulseLW;
            checkEncoderRW = Encoders::pulseRW;
        }

        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, driveDuty, 30);

        // drive fwd for many cms
        Encoders::driveMotorsDistance(driveDuty, true, driveFwd);

        // rotate left at an angle to treasure
        Motors::RotateMode rotateMode = Motors::RotateMode::BOTH_WHEELS;

        if (cache) Encoders::startAddActionCache(Motors::MotorAction::ROTATE_LEFT, rotateMode, driveDuty);
        Encoders::rotateMotorsDegs(driveDuty, false, rotateMode, rotateLeftDegs);
        if (cache) Encoders::endAddActionCache();

        if (cache) Encoders::startAddActionCache(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        // drive to treasure
        driveToTreasureFrontSonarIR3(31, claw_req_good_readings[2], timeout);

        Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 4.5);
        // too close to the treasure
        // driveBackToTreasureFrontSonar(claw_req_good_readings[2]);

        if (cache) Encoders::endAddActionCache();
        
        // collect
        Servos::collectTreasure();
        // Servos::collectTreasureUsingInterrupt();

        // return to original location
        if (cache) Encoders::executeReverseCache();

        return true;
    }

    bool obtainIRTreasure(int treasureNum, bool cache) {
        // look for treasure
        int goodSideSonarReadings = 0;
        double rightSonarDist;
        long startMillis = millis();
        long currMillis = startMillis;
        // while (goodSideSonarReadings < side_sonar_req_good_readings[treasureNum - 1]) {
        // last good reading
        while (true) {
            IR::driveWithPID();
            if (currMillis > startMillis + 35) {
                rightSonarDist = Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT, 0);
                startMillis = currMillis;
            }
            currMillis = millis();

            Serial.print("Right sonar dist: ");
            Serial.println(rightSonarDist);

            // TODO maybe bad readigns
            // consecutive good readings
            if (rightSonarDist < side_sonar_treasure_dists[treasureNum-1] + side_sonar_treasure_dists_err[treasureNum-1] && rightSonarDist > side_sonar_treasure_dists[treasureNum-1] - side_sonar_treasure_dists_err[treasureNum-1]) {
                goodSideSonarReadings++;
            } else {
                // last good reading
                if (goodSideSonarReadings >= side_sonar_req_good_readings[treasureNum-1]) {
                    break;
                } else {
                    goodSideSonarReadings = 0;
                }
            }            
        }
        // stop
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::dutyCycleL, 50, Motors::default_motors_stop_millis, Motors::dutyCycleR - Motors::dutyCycleL);

        Serial.println("----------- Treasure right -------: ");
       
        // which sonar to check
        Sonars::SonarType sonarType = Sonars::SonarType::RIGHT;

        // found treasure, move up a bit
        int driveDuty = 40;
        Encoders::driveMotorsDistance(driveDuty, true, 23);
        if (!Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, 50, 2)) {
            // timed out, hit pedestal when turning
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 7, 1);
        }
        Encoders::driveMotorsDistance(driveDuty, false, 18.5, 3);
        // Encoders::driveMotorsDistance(driveDuty, true, 10, 3);
        return treasureCollectionRoutine(sonarType, front_sonar_treasure_dists[treasureNum-1], front_sonar_treasure_dists_err[treasureNum-1], cache, treasureNum);
    }
    
    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum) {
        // drive back a bit if needed
        if (treasureNum == 1) {
            int driveCalibration = 6;    // cm
            Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, driveCalibration);
        }

        double avgTreasureFrontSonarDists = 0;  

        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {

            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            Motors::RotateMode treasureTurnRotateMode = Motors::RotateMode::BOTH_WHEELS;
            
            // 2nd treasure is special - overshoots right rotation first
            if (treasureNum == 2) {
                treasureTurnAction = Motors::ROTATE_LEFT;
                treasureTurnRotateMode = Motors::RotateMode::FORWARDS; 
            } else if (treasureNum == 5) {
                treasureTurnRotateMode = Motors::RotateMode::BOTH_WHEELS;
            }
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm+20);

            // turn until certain within threshold
            double treasureFrontSonarDists;
            if (treasureNum == 1 || treasureNum == 4) {
                Motors::rotate(Motors::min_rotate_dutyCycle, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            } else {
                Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            }
            
            long startMillis = millis();
            long currMillis = startMillis;
            
            int goodReadingsCount = 0;
            while (goodReadingsCount < front_sonar_req_good_readings[treasureNum-1]) {
                if (currMillis > startMillis + 35) {
                    treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT, 0);
                    startMillis = currMillis;
                }
                currMillis = millis();

                if (treasureFrontSonarDists < distFront + distFrontErr && treasureFrontSonarDists > distFront - distFrontErr) {
                    goodReadingsCount++;
                    avgTreasureFrontSonarDists += treasureFrontSonarDists;
                }
                else {
                    goodReadingsCount = 0;
                }
                Serial.print("Front sonar: ");
                Serial.println(treasureFrontSonarDists);
            }
            avgTreasureFrontSonarDists /= (1.0 * front_sonar_req_good_readings[treasureNum-1]);

            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm, 100);

            // empirical - 2nd treasure requires some degs to be rotated to the left to be aligned to treasure
            if (treasureNum == 2) {
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 12);
            }
            if (retOriginalPos) Encoders::endAddActionCache();

        } else {
            // treasure initially in front
            avgTreasureFrontSonarDists = Sonars::getAvgDistancePulses(10, Sonars::SonarType::FRONT);
        }

        // fourth one special
        if (treasureNum == 4) {
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, 8);
        }
        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        if (treasureNum == 4) {
            // drive in slowly for 4th treasure
            driveToTreasureFrontSonar(avgTreasureFrontSonarDists, treasureNum, 2.5, false, Motors::min_drive_dutyCycle); 
        } else {
            driveToTreasureFrontSonar(avgTreasureFrontSonarDists, treasureNum, 1.5, false);   // this backs up initially - don't cache, done externally
        }
        // too close to the treasure
        driveBackToTreasureFrontSonar(treasureNum);
        
        if (retOriginalPos) Encoders::endAddActionCache();    
    
        // collect
        Servos::collectTreasure();
        // Servos::collectTreasureUsingInterrupt();

        // return to original position
        if (retOriginalPos) Encoders::executeReverseCache();

        return true;
    }

    void regularDriveToTreasureFront(int treasureNum, int timeout) {
        int goodReadingsCount = 0;
        double treasureFrontSonarDists;
        Motors::driveFwd(def_drive_to_treasure_duty);

        long startMillis = millis();
        long currMillis = startMillis;

        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            // Serial.print("Front sonar dist: ");
            // Serial.println(frontSonarTreasureInClawDists);            
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            if (treasureFrontSonarDists < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;

            if (currMillis > startMillis + timeout*1000) {
                break;
            }
            currMillis = millis();
        }
        
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 50);
    }
    
    void driveBackToTreasureFrontSonar(int treasureNum) {
        int reqGoodReadings = claw_req_good_readings[treasureNum-1];

        // too close to the treasure
        Motors::driveBack(def_drive_to_treasure_duty);
        int goodReadingsCount = 0;
        double frontSonarTreasureInClawDists;
        while (goodReadingsCount < reqGoodReadings) {
            frontSonarTreasureInClawDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            Serial.print("Front sonar dist: ");
            Serial.println(frontSonarTreasureInClawDists);            

            if (frontSonarTreasureInClawDists > treasure_in_claw_dist - treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 100);

        Serial.print("found treasure claw: ");    
        Serial.println(frontSonarTreasureInClawDists);   
    }

    void driveToTreasureFrontSonar(double initialDist, int treasureNum, int timeout, bool retOriginalPos, int dutyCycle) {
        if (retOriginalPos) Encoders::startAddActionCache(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, dutyCycle);
        // drive fwd while reading sonar
        // Motors::driveFwd(def_drive_to_treasure_duty);

        // drive fwd until treasure inside V
        Encoders::driveMotorsDistance(dutyCycle, true, 27, timeout);
        // delay to check sonar readings reliably
        delay(300);

        // check readings
        // Motors::driveFwd(def_drive_to_treasure_duty);

        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double treasureFrontSonarDists;


        // needs 4 out of 10 readings within the V distance to be considered as good
        for (int i = 0; i < 20; i++) {
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            if (treasureFrontSonarDists > treasure_in_v_dist) {
                needsCalibrationCount++;
            } 
            if (needsCalibrationCount >= 15) {
                needsCalibrationFlag = true;
                break;
            }            
            Serial.print("i : ");
            Serial.print(i);
            Serial.print(" dist: ");
            Serial.println(treasureFrontSonarDists);
        }
        if (needsCalibrationFlag) {
            bool doubleCalibration = false;
            Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, 1);
            delay(300);
            for (int i = 0; i < 20; i++) {
                treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
                if (treasureFrontSonarDists > treasure_in_v_dist) {
                    needsCalibrationCount++;
                } 
                if (needsCalibrationCount >= 15) {
                    doubleCalibration = true;
                    break;
                }            
                Serial.print("i : ");
                Serial.print(i);
                Serial.print(" dist: ");
                Serial.println(treasureFrontSonarDists);
            }
            if (!doubleCalibration) {
                needsCalibrationFlag = false;
            }
        }
        if (needsCalibrationFlag) {
            double backUpDist = 11;
            int turnDegs = 30;
            double expectedMaxDist = 28;
            if (treasureCalibration(backUpDist, turnDegs, expectedMaxDist, treasureNum)) {
                // found treasure post calibration - in line with treasure
                // drive in with same amount driven out
                Encoders::driveMotorsDistance(dutyCycle, true, backUpDist, 3);

                // drive out to claw distance
                Encoders::driveMotorsDistance(dutyCycle, false, v_to_claw_dist);
            }
        } else {
            // does not need calibration - drive back to claw range
            Encoders::driveMotorsDistance(dutyCycle, false, v_to_claw_dist);
        }
        if (retOriginalPos) Encoders::endAddActionCache();
    }

    void driveToTreasureFrontSonarIR3(double driveFwdDist, int reqGoodReadings, int timeout) {        
        // drive fwd while reading sonar
        // Motors::driveFwd(def_drive_to_treasure_duty);

        // drive fwd until treasure inside V
        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, driveFwdDist, 1);
        // delay to check sonar readings reliably
        delay(300);

        // check readings
        // Motors::driveFwd(def_drive_to_treasure_duty);
        
        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double treasureFrontSonarDists;

        for (int i = 0; i < 10; i++) {
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            if (treasureFrontSonarDists > treasure_in_claw_dist) {
                needsCalibrationCount++;
            }
            if (needsCalibrationCount >= 6) {
                needsCalibrationFlag = true;
                break;
            }            
        }

        long startMillis = millis();
        long currMillis = startMillis;
    
        // needs calibration
        if (needsCalibrationFlag) {
            double backUpDist = 6.5;
            double turnDegs = 40;
            double expectedMaxDist = 24;
            if (treasureCalibration(backUpDist, turnDegs, expectedMaxDist, reqGoodReadings)) {
                // found treasure after calibration - in line w/ treasure
                // drive into V since now in line w/ treasure
                Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, backUpDist, 3);  
            }
        }
        // does not need calibration - drive back to claw range
        // Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, v_to_claw_dist);
    }

    bool treasureCalibration(double backUpDist, double turnDegs, double expectedMaxDist, int treasureNum) {
        int reqGoodReadings = claw_req_good_readings[treasureNum-1];
        Motors::RotateMode mode = Motors::RotateMode::BOTH_WHEELS;
        
        // back up a bit
        Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, backUpDist);

        // first turn left until completely off pedestal
        Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, mode, turnDegs);

        // turn right until certain within threshold
        Motors::rotate(Motors::min_rotate_dutyCycle, true, mode);
        double radius = mode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
        
        int turnPulseInterval = 2 * Encoders::degsToPulses(turnDegs, Motors::WHEELS_WIDTH);

        double treasureFrontSonarDists;
        int goodReadingsCount = 0;
        bool found = false;
        long pulsesStart = mode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
        long checkPulses = pulsesStart; 

        while (true) {
            // end condition - rotating right twice the amount of initial left turn
            if (checkPulses > pulsesStart + turnPulseInterval) {
                break;
            }
            // read dist
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            Serial.print("(Calibration) front sonar dists: ");
            Serial.println(treasureFrontSonarDists);
            // within threshold
            if (treasureFrontSonarDists < expectedMaxDist) {
                goodReadingsCount++;
            }
            else {
                goodReadingsCount = 0;
            }
            // within 'V'
            if (goodReadingsCount >= reqGoodReadings) {
                Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::default_rotate_pwm, 50);
                Serial.println("(Calibration) Found treasure");

                // // drive into to V
                // Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, backUpDist, 3);
                
                // // out to claw distance
                // Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, v_to_claw_dist);
                return true;
            }
            // update rotation
            checkPulses = mode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
        }
        Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::min_rotate_dutyCycle, 50);
        Encoders::rotateMotorsDegs(30, false, mode, turnDegs);

        return false;   // didn't find treasure
    }

    bool obtainFifthTreasure(int driveDuty, double rotateRightDegs, double driveBackDist, bool cache) {
        // driveBackDist = 25;
        // driveBackDistErr = 5;

        // manuveur so we can start treasure scanning front treasure
        Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, rotateRightDegs, 2);
        Encoders::driveMotorsDistance(driveDuty, false, driveBackDist);
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle, 30, 50);
        Encoders::driveMotorsDistance(driveDuty, true, 2.5);
        treasureCollectionRoutine(Sonars::SonarType::RIGHT, front_sonar_treasure_dists[5], front_sonar_treasure_dists_err[5], false, 5);

        // back up some cm
        Encoders::driveMotorsDistance(driveDuty, false, 1.5);

        /// below should be drop treasures func ///
        // turn to line up to drop treasures
        Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::BACKWARDS, 20);
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle, 40, 50);
        Encoders::driveMotorsDistance(40, true, 6, 1);
        Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::RotateMode::FORWARDS, 40);
        Encoders::driveMotorsDistance(driveDuty, false, 5.5);
        delay(200);
        Servos::deployBox();
        delay(400);
        Encoders::driveMotorsDistance(80, false, 3, 1);
        Encoders::driveMotorsDistance(50, true, 5, 1);
        
        return true;
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */

/* Ensuring driving into claw range
 * 1) drive until claw in range. If distance gets larger then rotate left and right until expected distance found
 * 2) drive until very close to pedestal (< 2 cm) using encoders & obtained sonar values. If the sonar value read is expected (~2 cm) then claw is in range.
 * Otherwise, back up some more cms and rotate some degs left & right until expected distance found (2+backed up cm)
 */
