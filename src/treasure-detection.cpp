#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"

namespace TreasureDetection {
    /* For sonar */
    // 6 potential treasures that needs to be located 
    const double side_sonar_treasure_dists[6] = {25, 28, 20, 20, 20, 20}; // cm
    const double side_sonar_treasure_dists_err[6] = {15, 12, 10, 2.5, 10, 10}; // cm

    const double front_sonar_treasure_dists[6] = {22, 30, 12.5, 15, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {20, 8, 10, 5}; // cm 

    const double treasure_in_claw_dist = 15; // cm
    const double treasure_in_claw_dist_err = 1; // cm

    extern const double treasure_in_v_dist = 14;

    const int side_sonar_req_good_readings[6] = {2, 2, 2, 2, 2, 2};
    const int front_sonar_req_good_readings[6] = {2, 2, 1, 1, 1, 1};
    const int claw_req_good_readings[6] = {2, 2, 2, 2, 2, 2};

    /* For encoder */
    const double v_to_claw_dist = 6;
    const double near_treasure_dists[6] = {120, 34};

    /* other */
    const double def_drive_to_treasure_duty = 30; // %


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

        // // only drive fwd 
        // Encoders::driveMotorsDistance(Motors::max_drive_dutyCycle, true, near_treasure_dists[treasureNum-1]);
        // // look for tape
        // TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::BOTH_WHEELS);

        // look for treasure
        while (goodSideSonarReadings < side_sonar_req_good_readings[treasureNum - 1]) {
            TapeFollow::driveWithPid();

            // only look for second treasure when passed chicken wire
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
        // Serial.println("found treasure right: ");    
        // Serial.println(rightSonarDist);    

        // 2nd treasure special: rotate until treasure not seen for second treasure
        if (treasureNum == 2) {
            double driveCalibration = 3;
            Encoders::startAddActionCache(Motors::DRIVE_BACK, Motors::NONE, Motors::min_drive_dutyCycle);
            // drive back a bit
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, driveCalibration);
            Encoders::endAddActionCache();

            // rotate right past treasure
            Encoders::startAddActionCache(Motors::ROTATE_RIGHT, Motors::BACKWARDS, Motors::default_rotate_pwm);
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BACKWARDS, 65);
            Encoders::endAddActionCache();
        }

        Sonars::SonarType sonarType;
        // which sonar to check
        if (treasureNum == 3) {
            sonarType = Sonars::SonarType::LEFT;
        } else {
            sonarType = Sonars::SonarType::RIGHT;
        }

        // cache
        bool retOriginal = true;
        if (treasureNum == 2) {
            retOriginal = false;
        }

        return treasureCollectionRoutine(sonarType, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, retOriginal, treasureNum);
    }

    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum) {
        
        // drive back a bit if needed
        int driveCalibration = 0;
        if (treasureNum == 1) {
                driveCalibration = 6;    // cm
                Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, driveCalibration);
        }

        double avgTreasureFrontSonarDists = 0;  

        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {            
            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            Motors::RotateMode treasureTurnRotateMode = Motors::RotateMode::BACKWARDS;
            // 2nd treasure is special - overshoots right rotation first
            if (treasureNum == 2) {
                treasureTurnAction = Motors::ROTATE_LEFT;
                treasureTurnRotateMode = Motors::RotateMode::FORWARDS; 
            }
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm);

            // turn until certain within threshold
            double treasureFrontSonarDists;

            int goodReadingsCount = 0;
            Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            
            while (goodReadingsCount < front_sonar_req_good_readings[treasureNum-1]) {
                treasureFrontSonarDists = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
                if (treasureFrontSonarDists < distFront + distFrontErr && treasureFrontSonarDists > distFront - distFrontErr) {
                    goodReadingsCount++;
                    avgTreasureFrontSonarDists += treasureFrontSonarDists;
                }
                else {
                    goodReadingsCount = 0;
                }
                Serial.println(treasureFrontSonarDists);
            }
            avgTreasureFrontSonarDists /= (1.0* front_sonar_req_good_readings[treasureNum-1]);

            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm, 100);

            // empirical - 2nd treasure requires some degs to be rotated to the left to be aligned to treasure
            if (treasureNum == 2) {
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 12);
            }

            if (retOriginalPos) Encoders::endAddActionCache();
        } else {
            // front sonar
            // avgTreasureFrontSonarDists = Sonars::getAvgDistancePulses(10, SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            avgTreasureFrontSonarDists = Sonars::getAvgDistancePulses(10, Sonars::SonarType::FRONT);
        }

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        /* Ensuring driving into claw range
         *  1) drive until claw in range. If distance gets larger then rotate left and right until expected distance found
         *  2) drive until very close to pedestal (< 2 cm) using encoders & obtained sonar values. If the sonar value read is expected (~2 cm) then claw is in range.
         *  Otherwise, back up some more cms and rotate some degs left & right until expected distance found (2+backed up cm)
         */

        driveToTreasureFrontSonar(avgTreasureFrontSonarDists, claw_req_good_readings[treasureNum-1], 4, retOriginalPos);   // this backs up initially
        // regularDriveToTreasureFront(treasureNum, 2.5);

        // too close to the treasure
        Motors::driveBack(def_drive_to_treasure_duty);
    
        int goodReadingsCount = 0;
        double treasureFrontSonarDistsTreasureClaw;
        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            treasureFrontSonarDistsTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            Serial.print("Front sonar dist: ");
            Serial.println(treasureFrontSonarDistsTreasureClaw);            

            if (treasureFrontSonarDistsTreasureClaw > treasure_in_claw_dist - treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_BACK, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 100);

        if (retOriginalPos) Encoders::endAddActionCache();
    
        Serial.println("found treasure claw: ");    
        Serial.println(treasureFrontSonarDistsTreasureClaw);   
        
        // collect
        Servos::collectTreasure();

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
            // Serial.println(treasureFrontSonarDistsTreasureClaw);            
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            if (treasureFrontSonarDists < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;

            if (currMillis > startMillis + timeout*1000) {
                break;
            }
            currMillis = millis();
        }
        
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 50);
    }

    void driveToTreasureFrontSonar(double initialDist, int reqGoodReadings, int timeout, bool retOriginalPos) {
        if (retOriginalPos) Encoders::startAddActionCache(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);
        // drive fwd while reading sonar

        Motors::driveFwd(def_drive_to_treasure_duty);

        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double treasureFrontSonarDists;

        long startMillis = millis();
        long currMillis = startMillis;
        
        while (goodReadingsCount < reqGoodReadings && !needsCalibrationFlag) {
            treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);
            Serial.print("Front sonar dist: ");
            Serial.println(treasureFrontSonarDists);            
            
            if (treasureFrontSonarDists < treasure_in_v_dist) goodReadingsCount++;
            else goodReadingsCount = 0;

            // readings larger than first reading
            if (treasureFrontSonarDists > initialDist) {
                needsCalibrationCount++;
            } else {
                needsCalibrationCount = 0;
            }
            if (needsCalibrationCount >= reqGoodReadings) {
                needsCalibrationFlag = true;
            }
            currMillis = millis();

            if (currMillis > startMillis + timeout * 1000) {
                needsCalibrationFlag = true;
            }
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::default_rotate_pwm, 50);
        
        if (retOriginalPos) Encoders::endAddActionCache();

        // needs calibration
        if (needsCalibrationFlag) {
            // back up a bit
            double backUpDist = 11;
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, backUpDist);

            Motors::RotateMode mode = Motors::RotateMode::BOTH_WHEELS;

            // first turn left until completely off pedestal
            int turnDegs = 30;
            Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, mode, turnDegs);

            Motors::rotate(Motors::min_rotate_dutyCycle, true, mode);
            double radius = mode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
            
            // turn right until certain within threshold
            int turnPulseInterval = 2 * Encoders::degsToPulses(turnDegs, Motors::WHEELS_WIDTH);
            Serial.println("Turn pulse: ");
            Serial.println(turnPulseInterval);

            double treasureFrontSonarDists;
            int goodReadingsCount = 0;
            bool found = false;
            long pulsesStart = mode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
            long checkPulses = pulsesStart; 

            double expectedMaxDist = 22;

            while (true) {
                // end condition - rotating right twice the amount of initial left turn
                if (checkPulses > pulsesStart + turnPulseInterval) {
                    break;
                }
                // read dist
                treasureFrontSonarDists = Sonars::getDistanceSinglePulse(Sonars::SonarType::FRONT);

                // within threshold
                if (treasureFrontSonarDists < expectedMaxDist) {
                    goodReadingsCount++;
                }
                else {
                    goodReadingsCount = 0;
                }

                Serial.println(treasureFrontSonarDists);
                // within 'V'
                if (goodReadingsCount >= reqGoodReadings) {
                    found = true;
                    Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::default_rotate_pwm, 50);
                    Serial.println("Found");

                    // drive into to V
                    Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, backUpDist, 3);
                    
                    // out to claw distance
                    Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, v_to_claw_dist);
                    return;
                }
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::min_rotate_dutyCycle, 50);
            // come back to original spot if not found
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, mode, turnDegs);
        }

        // does not need calibration - drive back to claw range
        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, v_to_claw_dist);
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */