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

    const double front_sonar_treasure_dists[6] = {24, 30, 24, 19, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {11, 14, 10, 10, 10, 7}; // cm 

    const double treasure_in_claw_dist = 14.5; // cm
    const double treasure_in_claw_dist_err = 1.5; // cm

    // extern const double treasure_in_v_dist = 12;
    extern const double treasure_in_v_dist = 14.5;
    extern const double treasure_in_v_rocks_dist = 19;

    const int side_sonar_req_good_readings[6] = {2, 2, 2, 3, 2, 2};
    const int front_sonar_req_good_readings[6] = {2, 2, 2, 2, 2, 2};
    const int claw_req_good_readings[6] = {2, 2, 2, 2, 2, 2};

    /* For encoder */
    const double v_to_claw_dist = 4.7;
    const double rock_v_to_claw_dist = 2.3;
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

        // for first treasure
        long startEncoderLW = Encoders::pulseLW;
        long startEncoderRW = Encoders::pulseLW;
        long checkEncoderLW = startEncoderLW;
        long checkEncoderRW = startEncoderRW;

        int rampDist = 140;
        int pulseIntervalRamp = Encoders::cmToPulses(rampDist);
        if (treasureNum == 1) Motors::max_drive_dutyCycle = 100;
        bool upRamp = false;
        bool checkSonar = treasureNum != 1;
        while (goodSideSonarReadings < side_sonar_req_good_readings[treasureNum - 1]) {
            if (!upRamp && treasureNum == 1) {
                checkEncoderLW = Encoders::pulseLW;                
                checkEncoderRW = Encoders::pulseRW;    
                if (!upRamp) {
                    if (checkEncoderLW > startEncoderLW + pulseIntervalRamp && checkEncoderRW > startEncoderRW) {
                        Motors::max_drive_dutyCycle = 60;
                        upRamp = true;
                    }            
                }
                if (upRamp) {
                    checkSonar = true;
                }
            }
            TapeFollow::driveWithPid();
                // only look for second treasure when passed chicken wire
            if (treasureNum == 2 && !TapeFollow::crossedChickenWire) rightSonarDist = -1;
            else {
                if (checkSonar) {
                    // sonar every 35 ms
                    if (currMillis > startMillis + 35) {
                        rightSonarDist = Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT, 0);
                        startMillis = currMillis; // update
                    } else {
                        rightSonarDist = -1;
                    }
                } else {
                    rightSonarDist = -1;
                }
            }
            currMillis = millis();
            
            Serial.println("Right sonar dist: ");
            Serial.println(rightSonarDist);

            ReflectanceSensors::printFrontReflectance();
            if (rightSonarDist != -1) {
                // consecutive good readings
                if (rightSonarDist < firstSideSonarTreausureDist + firstSideSonarTreausureDistErr && rightSonarDist > firstSideSonarTreausureDist - firstSideSonarTreausureDistErr) {
                    goodSideSonarReadings++;
                } else {
                    goodSideSonarReadings = 0;
                }
            }
            // if inf loop --> return false (some sort of timeout)
            // if (treasureNum == 1 && TapeFollow::crossedChickenWire) {
            //     return false;
            // }
        }
        // Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, LW_PWM_DUTY, 50);
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::dutyCycleL + 30, 100, Motors::default_motors_stop_millis, Motors::dutyCycleR - Motors::dutyCycleL + 30);

        Serial.print("Found treasure right: ");    
        Serial.println(rightSonarDist);    

        // 2nd treasure special: rotate until treasure not seen for second treasure
        if (treasureNum == 2) {
            double driveCalibration = 5;
            if (retToOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_BACK, Motors::NONE, 50);
            // drive back a bit
            Encoders::driveMotorsDistance(30, false, driveCalibration);
            if (retToOriginalPos) Encoders::endAddActionCache();
 
            // rotate right past treasure
            if (retToOriginalPos) Encoders::startAddActionCache(Motors::ROTATE_RIGHT, Motors::BACKWARDS, Motors::default_rotate_pwm+20);
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BACKWARDS, 65);
            if (retToOriginalPos) Encoders::endAddActionCache();
        }

        Sonars::SonarType sonarType = Sonars::SonarType::RIGHT;
        // finished colleted first treasure - increase duty cycle back
        if (treasureNum == 1) {
            Motors::max_drive_dutyCycle = 83;
        }
        return treasureCollectionRoutine(sonarType, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, retToOriginalPos, treasureNum);
    }

    void driveBackToTreasureFrontSonar(int, bool);

    bool obtainThirdIRtreasure(double driveFwd, double rotateLeftDegs, int driveDuty, bool cache) {
        int timeout = 2.7;
    
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

        double IRdriveDist = 26;
        int IRpulsesInterval = Encoders::cmToPulses(IRdriveDist);
        
        long startIRmillis = millis();
        long checkIRmillis = startIRmillis;
        int IRtimeoutMillis = 3500;
        bool IRtimeoutFlag = false;
        while (checkEncoderLW < startEncoderLW + IRpulsesInterval && checkEncoderRW < startEncoderRW + IRpulsesInterval) {
            IR::driveWithPID();
            checkEncoderLW = Encoders::pulseLW;
            checkEncoderRW = Encoders::pulseRW;
            // give a little nudge if not driving fwd
            if (!IRtimeoutFlag && checkIRmillis > startIRmillis + IRtimeoutMillis) {
                IRtimeoutFlag = true;
                Encoders::driveMotorsDistance(60, true, 6);
            }
            checkIRmillis = millis();
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
        bool inRocks = !driveToTreasureFrontSonar(38, claw_req_good_readings[2], timeout, false, def_drive_to_treasure_duty);
        driveBackToTreasureFrontSonar(3, inRocks);
        // Encoders::driveMotorsDistance(TreasureDetection::def_drive_to_treasure_duty, false, 4.5);

        // third treasure special
        Encoders::driveMotorsDistance(25, true, 1, 1);

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
        int distToFourthTreasure = 65;
        int toFourthTreasurePulseInterval = Encoders::cmToPulses(distToFourthTreasure);
        long startPulseLW = Encoders::pulseLW;
        long startPulseRW = Encoders::pulseRW;
        long checkPulseLW = startPulseLW;
        long checkPulseRW = startPulseRW;
        long hasNotFoundUsingSonar = false;
        while (true) {
            // end condition
            if (checkPulseLW > startPulseLW + toFourthTreasurePulseInterval && checkPulseRW > startPulseRW + toFourthTreasurePulseInterval) {
                hasNotFoundUsingSonar = true;
                break;
            }
            checkPulseLW = Encoders::pulseLW;
            checkPulseRW = Encoders::pulseRW;

            IR::driveWithPID();
            
            if (currMillis > startMillis + 35) {
                rightSonarDist = Sonars::getDistanceSinglePulse(Sonars::SonarType::RIGHT, 0);
                startMillis = currMillis;   // update time
            } else {
                rightSonarDist = -1;
            }
            currMillis = millis();
            
            Serial.print("Right sonar dist: ");
            Serial.println(rightSonarDist);

            // TODO maybe bad readigns
            // consecutive good readings
            if (rightSonarDist != -1) {
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
        }
        // stop
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::dutyCycleL+10, 50, 150, Motors::dutyCycleR - Motors::dutyCycleL+10);

        Serial.println("----------- Treasure right -------: ");
       
        // which sonar to check
        Sonars::SonarType sonarType = Sonars::SonarType::RIGHT;

        // found treasure, move up a bit
        int driveDuty = 60;
        // drive fwd less if ended using encoders - already drives more
        double initRightRotation;
        if (hasNotFoundUsingSonar) {
            Encoders::driveMotorsDistance(driveDuty, true, 6.5);
            initRightRotation = 53.1;
        } else {
            Encoders::driveMotorsDistance(driveDuty, true, 20.5);
            initRightRotation = 53.1;

        }
        if (!Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, initRightRotation, 2)) {
            // timed out, hit pedestal when turning
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::BOTH_WHEELS, 8, 1);
        }
        Encoders::driveMotorsDistance(driveDuty, false, 19.7, 3);
        // Encoders::driveMotorsDistance(driveDuty, true, 10, 3);
        return treasureCollectionRoutine(sonarType, front_sonar_treasure_dists[treasureNum-1], front_sonar_treasure_dists_err[treasureNum-1], cache, treasureNum);
    }
    
    bool obtainFifthTreasure(int driveDuty, double rotateRightDegs, double driveBackDist, bool cache) {
        // driveBackDist = 25;
        // driveBackDistErr = 5;
        // for(int i = 0; i < 10; i++) {
        //     Serial.println("Hit");
        //     delay(300);
        // }
        // manuveur so we can start treasure scanning front treasure
        if (!Encoders::rotateMotorsDegs(40, true, Motors::RotateMode::BOTH_WHEELS, rotateRightDegs, 2)) {
            Encoders::rotateMotorsDegs(40, false, Motors::RotateMode::BOTH_WHEELS, 10, 2);
        }
        Encoders::driveMotorsDistance(driveDuty, false, driveBackDist);
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle+10, 50, 100);
        Encoders::driveMotorsDistance(driveDuty, true, 3.5);

        treasureCollectionRoutine(Sonars::SonarType::RIGHT, front_sonar_treasure_dists[4], front_sonar_treasure_dists_err[4], false, 5);
        // very last treasure - open claw so box can be opened
        Servos::clawServo.write(Servos::claw_full_open_angle);

        // back up some cm
        Encoders::driveMotorsDistance(driveDuty, false, 1.5);

        /// below should be drop treasures func ///
        // turn to line up to drop treasures
        Encoders::rotateMotorsDegs(50, false, Motors::RotateMode::BACKWARDS, 30, 1);
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle+10, 40, 50);
        Encoders::driveMotorsDistance(60, true, 7.5, 1);
        Encoders::rotateMotorsDegs(50, false, Motors::RotateMode::BOTH_WHEELS, 31.4);
        Encoders::driveMotorsDistance(driveDuty, false, 6, 2);
        Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle, 60, 50);
        Encoders::driveMotorsDistance(60, true, 0.5, 1);
        Servos::deployBox();
        delay(500);
        Encoders::driveMotorsDistance(60, true, 3, 1);
        Encoders::driveMotorsDistance(60, false, 2.7, 1);
        for (int i = 0; i < 10; i++) {
            Encoders::rotateMotorsDegs(30, true, Motors::RotateMode::BOTH_WHEELS, 20, 1);
            Encoders::driveMotorsDistance(50, true, 10.3, 1);
            Encoders::driveMotorsDistance(80, false, 10, 1);
        }
        Encoders::driveMotorsDistance(60, true, 5, 1);

        return true;
    }

    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum) {
        // drive back a bit if needed
        if (treasureNum == 1) {
            int driveCalibration = 7.5;    // cm
            Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, driveCalibration);
        }

        double avgTreasureFrontSonarDists = 0;  

        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {

            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            Motors::RotateMode treasureTurnRotateMode = Motors::RotateMode::BOTH_WHEELS;
            
            // 2nd treasure is special - overshoots right rotation first
            if (treasureNum == 1) {
                treasureTurnRotateMode = Motors::RotateMode::BACKWARDS;
            }
            if (treasureNum == 2) {
                treasureTurnAction = Motors::MotorAction::ROTATE_LEFT;
                treasureTurnRotateMode = Motors::RotateMode::FORWARDS; 
            } else if (treasureNum == 5) {
                treasureTurnRotateMode = Motors::RotateMode::BOTH_WHEELS;
            }
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm+20);

            // turn until certain within threshold
            double treasureFrontSonarDists;
            if (treasureNum == 1 || treasureNum == 4 || treasureNum == 5) {
                Motors::rotate(Motors::min_rotate_dutyCycle, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            } else {
                Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            }

            long startMillis = millis();
            long currMillis = startMillis;
            
            int sonarDelayMillis = 35;
            if (treasureNum == 4) sonarDelayMillis = 60;

            int goodReadingsCount = 0;
            
            long startEncoderPulses;
            long checkEncoderPulses;
            
            if (treasureTurnAction == Motors::MotorAction::ROTATE_LEFT) {
                if (treasureTurnRotateMode == Motors::RotateMode::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseLW;
                } else {
                    startEncoderPulses = Encoders::pulseRW;
                }
            } else {
                if (treasureTurnRotateMode == Motors::RotateMode::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseRW;
                } else {
                    startEncoderPulses = Encoders::pulseLW;
                }   
            }
            checkEncoderPulses = startEncoderPulses;

            int maxRotAngle;
            int turnBackAngle;
            bool maxRotReached = false;

            if (treasureNum == 1) {
                maxRotAngle = 65;
                turnBackAngle = 32;
            } else if (treasureNum == 2) {
                maxRotAngle = 90;
                turnBackAngle = 50;
            } else if (treasureNum == 4) {
                maxRotAngle = 50;
                turnBackAngle = 32;
            } else if (treasureNum == 5) {
                maxRotAngle = 55;
                turnBackAngle = 38;
            }
            double radius = treasureTurnRotateMode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
            int maxRotTurnPulses = Encoders::degsToPulses(maxRotAngle, radius);

            while (goodReadingsCount < front_sonar_req_good_readings[treasureNum-1]) {
                if (currMillis > startMillis + sonarDelayMillis) {
                    treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT, 0);
                    startMillis = currMillis;
                    Serial.print("Front sonar: ");
                    Serial.println(treasureFrontSonarDists);
                } else {
                    treasureFrontSonarDists = -1;
                }
                currMillis = millis();
                if (treasureFrontSonarDists != -1) {
                    if (treasureFrontSonarDists < distFront + distFrontErr && treasureFrontSonarDists > distFront - distFrontErr) {
                        goodReadingsCount++;
                        avgTreasureFrontSonarDists += treasureFrontSonarDists;
                    }
                    else {
                        goodReadingsCount = 0;
                    }
                }
                
                // update encoder
                if (treasureTurnAction == Motors::MotorAction::ROTATE_LEFT) {
                    if (treasureTurnRotateMode == Motors::RotateMode::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseLW;
                    } else {
                        checkEncoderPulses = Encoders::pulseRW;
                    }
                } else {
                    if (treasureTurnRotateMode == Motors::RotateMode::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseRW;
                    } else {
                        checkEncoderPulses = Encoders::pulseLW;
                    }   
                } 
                // end condition - max rotation)
                if (maxRotReached) {
                    break;
                }
                if (treasureTurnRotateMode == Motors::RotateMode::BACKWARDS) {
                    // pulses decrease
                    if (checkEncoderPulses < startEncoderPulses - maxRotTurnPulses) {
                        maxRotReached = true;
                    }
                } else {
                    // pulses increase
                    if (checkEncoderPulses > startEncoderPulses + maxRotTurnPulses) {
                        maxRotReached = true;
                    }
                }
                
            }
            avgTreasureFrontSonarDists /= (1.0 * front_sonar_req_good_readings[treasureNum-1]);
            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm, 100);
            // if sonar misses - go other direction
            if (maxRotReached) {
                Motors::MotorAction turnBackAction;
                Motors::RotateMode turnBackRotationMode;
                std::tie(turnBackAction, turnBackRotationMode) = Motors::getInverseDrive(treasureTurnAction, treasureTurnRotateMode);
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, turnBackAction == Motors::MotorAction::ROTATE_RIGHT, turnBackRotationMode, turnBackAngle, 2);
            }

            // empirical - 2nd treasure requires some degs to be rotated to the left to be aligned to treasure
            if (treasureNum == 2) {
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 13);
            }
            if (retOriginalPos) Encoders::endAddActionCache();

        } else {
            // treasure initially in front
            avgTreasureFrontSonarDists = Sonars::getAvgDistancePulses(10, Sonars::SonarType::FRONT);
        }

        // special
        if (treasureNum == 1) {
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, 4);
        }
        if (treasureNum == 4) {
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, 3.5);
        } else if (treasureNum == 5) {
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BOTH_WHEELS, 4.3);
        } 

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        bool inRocks;
        int driveToTreasureDuty;
        int timeoutTreasure;
        if (treasureNum == 4 || treasureNum == 5) {
            // drive in slowly for 4th treasure
            // driveToTreasureDuty = 35;
            driveToTreasureDuty = 28;
        } else {
            // driveToTreasureDuty = 45;
            driveToTreasureDuty = 35;
        }
        if (treasureNum == 3 || treasureNum == 5) {
            // timeoutTreasure = 1;
            timeoutTreasure = 1.35;
        } else {
            // timeoutTreasure = 1.4;
            timeoutTreasure = 1.8;
        }
        inRocks = !driveToTreasureFrontSonar(avgTreasureFrontSonarDists, treasureNum, timeoutTreasure, false, driveToTreasureDuty);
        // too close to the treasure
        driveBackToTreasureFrontSonar(treasureNum, inRocks);
        
        if (retOriginalPos) Encoders::endAddActionCache();    
    
        // collect
        Servos::collectTreasure();
        // Servos::collectTreasureUsingInterrupt();

        // return to original position
        if (retOriginalPos) Encoders::executeReverseCache();

        return true;
    }

    void driveBackToTreasureFrontSonar(int treasureNum, bool inRocks) {
        int reqGoodReadings = claw_req_good_readings[treasureNum-1];
        // drive back
        double driveBackDist;
        if (!inRocks) {
            driveBackDist = v_to_claw_dist;
        } else {
            driveBackDist = rock_v_to_claw_dist;
        }
        Encoders::driveMotorsDistance(40, false, driveBackDist); 
    }

    bool driveToTreasureFrontSonar(double initialDist, int treasureNum, int timeout, bool retOriginalPos, int dutyCycle) {
        if (retOriginalPos) Encoders::startAddActionCache(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, dutyCycle);
        // drive fwd while reading sonar
        // Motors::driveFwd(def_drive_to_treasure_duty);

        // drive fwd until treasure inside V
        Encoders::driveMotorsDistance(dutyCycle, true, 43, timeout);
        // delay to check sonar readings reliably

        // check readings
        // Motors::driveFwd(def_drive_to_treasure_duty);

        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double treasureFrontSonarDists;
        
        int inVCount = 0;
        int inRocksCount = 0;

        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, 1);
        
        // needs 4 out of 10 readings within the V distance to be considered as good
        for (int i = 0; i < 20; i++) {
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            // if (treasureFrontSonarDists > treasure_in_v_dist) {
            if (treasureFrontSonarDists > treasure_in_v_rocks_dist) {   // worst of the two
                needsCalibrationCount++;
            } 
            if (treasureFrontSonarDists > treasure_in_v_dist && treasureFrontSonarDists < treasure_in_v_rocks_dist) { 
                inRocksCount++;
            } 
            if (treasureFrontSonarDists < treasure_in_v_dist) {   // worst of the two
                inVCount++;
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
        bool doubleCalibrationFlag = false;

        if (needsCalibrationFlag && treasureNum != 3) {
            delay(250);
            for (int i = 0; i < 20; i++) {
                treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
                // if (treasureFrontSonarDists > treasure_in_v_dist) {
                if (treasureFrontSonarDists > treasure_in_v_rocks_dist) {
                    needsCalibrationCount++;
                } 
                if (needsCalibrationCount >= 15) {
                    doubleCalibrationFlag = true;
                    break;
                }            
                Serial.print("i : ");
                Serial.print(i);
                Serial.print(" dist: ");
                Serial.println(treasureFrontSonarDists);
            }
        }
        if (doubleCalibrationFlag && treasureNum != 3) {
            double backUpDist = 11;
            int turnDegs = 30;
            double expectedMaxDist = 28;
            if (treasureNum == 3) {
                backUpDist = 6.5;
                turnDegs = 40;
                expectedMaxDist = 24;
            }
            if (treasureNum == 5) {
                expectedMaxDist = 30;
            }
            if (treasureCalibration(backUpDist, turnDegs, expectedMaxDist, treasureNum)) {
                // found treasure post calibration - in line with treasure
                // drive in with more than amount driven out
                Encoders::driveMotorsDistance(dutyCycle, true, backUpDist+10, 1.8);

                // drive out to claw distance
                // Encoders::driveMotorsDistance(dutyCycle, false, v_to_claw_dist);
            }
        } else {
            // does not need calibration - drive back to claw range
            // Encoders::driveMotorsDistance(dutyCycle, false, v_to_claw_dist);
        }
        if (retOriginalPos) Encoders::endAddActionCache();
        return inVCount >= inRocksCount;
    }

    bool driveToTreasureFrontSonarIR3(double driveFwdDist, int reqGoodReadings, int timeout) {        
        // drive fwd while reading sonar
        // Motors::driveFwd(def_drive_to_treasure_duty);

        // drive fwd until treasure inside V
        Encoders::driveMotorsDistance(30, true, driveFwdDist, 1);
        // delay to check sonar readings reliably
\
        // check readings
        // Motors::driveFwd(def_drive_to_treasure_duty);
        
        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double treasureFrontSonarDists;
        
        int inVCount = 0;
        int inRocksCount = 0;
        
        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, 1);

        delay(100);

        for (int i = 0; i < 20; i++) {
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            if (treasureFrontSonarDists > treasure_in_v_rocks_dist) {   // worst of the two
                needsCalibrationCount++;
            } 
            if (treasureFrontSonarDists > treasure_in_v_dist && treasureFrontSonarDists < treasure_in_v_rocks_dist) { 
                inRocksCount++;
            } 
            if (treasureFrontSonarDists < treasure_in_v_dist) {   // worst of the two
                inVCount++;
            } 
            if (treasureFrontSonarDists > treasure_in_claw_dist) {
                needsCalibrationCount++;
            }
            if (needsCalibrationCount >= 15) {
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
                Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, backUpDist+8, 1.5);  
            }
        }
        // does not need calibration - drive back to claw range
        // Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, v_to_claw_dist);
        return inVCount >= inRocksCount;
    }

    bool treasureCalibration(double backUpDist, double turnDegs, double expectedMaxDist, int treasureNum) {
        int reqGoodReadings = claw_req_good_readings[treasureNum-1];
        Motors::RotateMode mode = Motors::RotateMode::BOTH_WHEELS;
        
        // back up a bit
        if (treasureNum == 5) {
            Motors::driveBackRearReflectance(Motors::min_drive_dutyCycle, 30, 50);
        } else {
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, backUpDist);
        }

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
