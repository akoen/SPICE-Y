#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"

namespace TreasureDetection {
    // 6 potential treasures that needs to be located 
    const double side_sonar_treasure_dists[6] = {25, 28, 20, 20, 20, 20}; // cm
    const double side_sonar_treasure_dists_err[6] = {15, 12, 10, 2.5, 10, 10}; // cm

    const double front_sonar_treasure_dists[6] = {22, 30, 12.5, 15, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {20, 8, 10, 5}; // cm 
    // const double front_sonar_treasure_dists[6] = {30, 28, 12.5, 15, 20, 20}; // cm
    // const double front_sonar_treasure_dists_err[6] = {20, 8, 10, 5}; // cm 

    const double treasure_in_claw_dist = 15; // cm
    const double treasure_in_claw_dist_err = 1; // cm

    const double def_drive_to_treasure_duty = Motors::min_drive_dutyCycle; // %

    const int side_sonar_req_good_readings[6] = {2, 2, 3, 3, 3, 3};
    const int front_sonar_req_good_readings[6] = {2, 2, 1, 1, 1, 1};
    const int claw_req_good_readings[6] = {2, 2, 3, 3, 3, 3};

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
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, LW_PWM_DUTY, 50, 1000);
        // Serial.println("found treasure right: ");    
        // Serial.println(rightSonarDist);    

        // rotate until treasure not seen for second treasure
        if (treasureNum == 2) {
            double driveCalibration = 3;
            Encoders::startAddActionCache(Motors::DRIVE_BACK, Motors::NONE, Motors::min_drive_dutyCycle);
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, driveCalibration);
            Encoders::endAddActionCache();
            Encoders::startAddActionCache(Motors::ROTATE_RIGHT, Motors::BACKWARDS, Motors::default_rotate_pwm);
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, true, Motors::RotateMode::BACKWARDS, 70);
            Encoders::endAddActionCache();
        }
        Sonars::SonarType sonarType;
        // which sonar to check
        if (treasureNum == 3) {
            sonarType = Sonars::SonarType::LEFT;
        } else {
            sonarType = Sonars::SonarType::RIGHT;
        }
        return treasureCollectionRoutine(sonarType, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, true, treasureNum);
    }

    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum) {
        // drive forward a bit if needed
        int driveCalibration = 0;
        if (treasureNum == 1) {
                driveCalibration = 6;    // cm - so wheel not caught on ramp
                // Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, true, fwdDriveCalibration);
                Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, driveCalibration);
        }
        double avgFoundTreasureFrontDists = 0;

        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {            
            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            if (treasureNum==2) {
                treasureTurnAction = Motors::ROTATE_LEFT;
            }
            Motors::RotateMode treasureTurnRotateMode = treasureNum == 2 ? Motors::RotateMode::FORWARDS : Motors::RotateMode::BACKWARDS;
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm);

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
                Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, Motors::RotateMode::FORWARDS, 8);
            }

            if (retOriginalPos) Encoders::endAddActionCache();
        }

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        /* Ensuring driving into claw range
         *  1) drive until claw in range. If distance gets larger then rotate left and right until expected distance found
         *  2) drive until very close to pedestal (< 2 cm) using encoders & obtained sonar values. If the sonar value read is expected (~2 cm) then claw is in range.
         *  Otherwise, back up some more cms and rotate some degs left & right until expected distance found (2+backed up cm)
         */
        // driveToTreasureFrontSonar(avgFoundTreasureFrontDists, claw_req_good_readings[treasureNum-1], 3);
        driveToTreasureFrontSonar2(avgFoundTreasureFrontDists, claw_req_good_readings[treasureNum-1], 4);   // this backs up initially
        // regularDriveToTreasureFront(treasureNum, 2.5);

        // too close to the treasure
        Motors::setDir(false, false);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();
        
        int goodReadingsCount = 0;
        double distFrontSonarTreasureClaw;
        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonarTreasureClaw);            

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
    
    void regularDriveToTreasureFront(int treasureNum, int timeout) {
        int goodReadingsCount = 0;
        double distFrontSonar;
        int dutyCycle = def_drive_to_treasure_duty;
        Motors::setDir(true, true);
        Motors::setDutyCycles(dutyCycle, dutyCycle);
        Motors::drive();

        long startMillis = millis();
        long currMillis = startMillis;
        while (goodReadingsCount < claw_req_good_readings[treasureNum-1]) {
            // Serial.print("Front sonar dist: ");
            // Serial.println(distFrontSonarTreasureClaw);            
            distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            if (distFrontSonar < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
            else goodReadingsCount = 0;

            if (currMillis > startMillis + timeout*1000) {
                break;
            }
            currMillis = millis();
        }
        
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, dutyCycle, 50);
    }

    void driveToTreasureFrontSonar2(double initialDist, int reqGoodReadings, int timeout) {
        // drive fwd while reading sonar
        Motors::setDir(true, true);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();

        int goodReadingsCount = 0;
        int needsCalibrationCount = 0;

        bool needsCalibrationFlag = false;
        double distFrontSonar;

        long startMillis = millis();
        long currMillis = startMillis;
        while (goodReadingsCount < reqGoodReadings && !needsCalibrationFlag) {
            distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonar);            
            // if (distFrontSonar < treasure_in_claw_dist + treasure_in_claw_dist_err) goodReadingsCount++;
            double rightInVdist = 11;
            if (distFrontSonar < rightInVdist) goodReadingsCount++;
            else goodReadingsCount = 0;

            // readings larger than first reading
            // if(distFrontSonar > initialDist) {
            //     needsCalibrationCount++;
            // }
            // if (needsCalibrationCount >= reqGoodReadings) {
            //     needsCalibrationFlag = true;
            // }

            currMillis = millis();
            if (currMillis > startMillis + timeout * 1000) {
                needsCalibrationFlag = true;
            }
        }
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::default_rotate_pwm, 50);
        
        // needs calibration
        if (needsCalibrationFlag) {
            // back up a bit
            double backUpDist = 11;
            Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, backUpDist);

            // look around
            Motors::RotateMode mode = Motors::RotateMode::BOTH_WHEELS;

            int turnDegs = 30;
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, mode, turnDegs);

            Motors::rotate(Motors::default_rotate_pwm, true, mode);
            double radius = mode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
            
            int turnPulseInterval = 2 * Encoders::degsToPulses(turnDegs, Motors::WHEELS_WIDTH);
            Serial.println("Turn pulse: ");
            Serial.println(turnPulseInterval);

            // turn right until certain within threshold
            double distFrontSonar;
            int goodReadingsCount = 0;
            bool found = false;
            long pulsesStart = mode == Motors::RotateMode::BACKWARDS ? Encoders::pulseRW : Encoders::pulseLW;
            long checkPulses = pulsesStart; 

            double expectedMaxDist = 20;
            while (true) {
                // end condition - rotating right twice the amount of initial left turn
                if (checkPulses > pulsesStart + turnPulseInterval) {
                    break;
                }
                // read dist
                distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

                // within threshold
                if (distFrontSonar < expectedMaxDist) {
                    goodReadingsCount++;
                }
                else {
                    goodReadingsCount = 0;
                }

                Serial.println(distFrontSonar);
                if (goodReadingsCount >= reqGoodReadings) {
                    found = true;
                    Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::default_rotate_pwm, 50);
                    Serial.println("Found");
                    // back up to claw range
                    // Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, 7);
                    // drive into to V
                    Encoders::driveMotorsDistance(def_drive_to_treasure_duty, true, backUpDist, 3);
                    // out to claw distance
                    double backUpFromVtoClawRange = 6;
                    Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, backUpFromVtoClawRange);
                    return;
                }
                Serial.print("Curr ");
                Serial.print(Encoders::pulseRW);
                Serial.print(" Start ");
                Serial.print(pulsesStart);
                Serial.print(" pulses interval ");
                Serial.print(turnPulseInterval);
                Serial.print(" ");
                Serial.print(turnPulseInterval);
                Serial.print(" ");
                Serial.print(turnPulseInterval);
                Serial.print(" ");
                Serial.println(turnPulseInterval);
            }
            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, mode, Motors::default_rotate_pwm, 50);
            // come back to original spot if not found
            Encoders::rotateMotorsDegs(Motors::default_rotate_pwm, false, mode, turnDegs);
            
            delay(1000);
        }
        // does not need calibration - drive back to claw range
        double backUpFromVtoClawRange = 6;
        Encoders::driveMotorsDistance(def_drive_to_treasure_duty, false, backUpFromVtoClawRange);
    }

    void driveToTreasureFrontSonar(double treasureDist, int numReadings, int timeout) {

        // drive forwards
        Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, true, treasureDist, timeout);

        // check sonar
        double expectedDistRange = 19;   // cm
        int numGoodReadings = 0;
        bool found = false;
        double avgSonarDist = Sonars::getAvgDistancePulses(numReadings, SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
        int turnAngle = 5;
        Serial.print("Avg sonar dists: ");
        Serial.println(avgSonarDist);
        // if missed - recalibrate
        if (avgSonarDist > expectedDistRange) {
            // back up a bit first
            Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, 5);
            // rotate left a bit
            Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::BOTH_WHEELS, turnAngle);
            // check readings
            double avgReadingsLeft = Sonars::getAvgDistancePulses(numGoodReadings, SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            
            // still bad, rotate back and right a bit
            if (avgReadingsLeft > expectedDistRange) {
                Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, true, Motors::BOTH_WHEELS, turnAngle);

                Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, true, Motors::BOTH_WHEELS, turnAngle);

                double avgReadingsRight = Sonars::getAvgDistancePulses(numGoodReadings, SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
                if (avgReadingsRight > expectedDistRange) {
                    // still bad, go back and just continue
                    Encoders::rotateMotorsDegs(Motors::min_rotate_dutyCycle, false, Motors::BOTH_WHEELS, turnAngle);
                }
            }
        }
        // move back to treasure distance
        // Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, treasureDist);
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */