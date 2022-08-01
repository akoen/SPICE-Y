#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"

namespace TreasureDetection {
    // 6 potential treasures that needs to be located 
    const double side_sonar_treasure_dists[6] = {20, 20, 20, 20, 20, 20}; // cm
    const double side_sonar_treasure_dists_err[6] = {12}; // cm
    const double front_sonar_treasure_dists[6] = {20, 20, 20, 20, 20, 20}; // cm
    const double front_sonar_treasure_dists_err[6] = {10}; // cm 
    const double treasure_in_claw_dist = 16; // cm
    const double treasure_in_claw_dist_err = 0; // cm

    const double def_drive_to_treasure_duty = 15; // %

    bool obtainTapeTreasure(int treasureNum) {
        // bad input
        if (treasureNum != 1 || treasureNum != 2) return false;

        Servos::configArmClawPins();

        double firstSideSonarTreausureDist = side_sonar_treasure_dists[treasureNum-1];
        double firstSideSonarTreausureDistErr = side_sonar_treasure_dists_err[treasureNum-1];
        
        double firstFrontSonarTreausureDist = front_sonar_treasure_dists[treasureNum-1];
        double firstFrontSonarTreausureDistErr = front_sonar_treasure_dists_err[treasureNum-1];

        // tape follow using PID until treasure located
        double loopCount = 0;
        double rightSonarDist = 0;
        do {        
            TapeFollow::driveWithPid();
            rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_R);

            Serial.println("Right sonar dist: ");
            Serial.println(rightSonarDist);

            ReflectanceSensors::printFrontReflectance();

            // if inf loop --> return false (some sort of timeout)
        } while (rightSonarDist > firstSideSonarTreausureDist + firstSideSonarTreausureDistErr || rightSonarDist < firstSideSonarTreausureDist - firstSideSonarTreausureDistErr);
        
        // Encoders::stopMotorsBrakeEncoders(Motors::DRIVE_FWD, Motors::NONE, Encoders::pulseLW, Encoders::pulseRW, LW_PWM_DUTY, 131);
        delay(10);
        Motors::stopWithBrake(Motors::DRIVE_FWD, Motors::NONE, LW_PWM_DUTY, 50);
        Serial.println("found treasure right: ");    
        Serial.println(rightSonarDist);    
        delay(1000);
        
        return treasureCollectionRoutine(Sonars::SonarType::RIGHT, firstFrontSonarTreausureDist, firstFrontSonarTreausureDistErr, true);
    }

    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos) {
        // turn until front sonar detects treasure
        if (treasureLoc == Sonars::SonarType::RIGHT || treasureLoc == Sonars::SonarType::LEFT) {
            Motors::MotorAction action = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT; 
            
            Motors::MotorAction treasureTurnAction = treasureLoc == Sonars::SonarType::RIGHT ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
            Motors::RotateMode treasureTurnRotateMode = Motors::RotateMode::BACKWARDS;
            
            if (retOriginalPos) Encoders::startAddActionCache(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm);
            
            Motors::rotate(Motors::default_rotate_pwm, treasureTurnAction == Motors::MotorAction::ROTATE_RIGHT, treasureTurnRotateMode);
            double distFrontSonar = 0;
            do {
                distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
                // TODO may need "if fail" handler
            } while(distFrontSonar > distFront + distFrontErr || distFrontSonar < distFront - distFrontErr);   
            // Encoders::stopMotorsBrakeEncoders(treasureTurnAction, treasureTurnRotateMode, Encoders::pulseLW, Encoders::pulseRW, Motors::default_rotate_pwm, 131);
            delay(1);
            Motors::stopWithBrake(treasureTurnAction, treasureTurnRotateMode, Motors::default_rotate_pwm+10, 200);
            if (retOriginalPos) Encoders::endAddActionCache();
        }

        // treasure in front
        if (retOriginalPos) Encoders::startAddActionCache(Motors::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty);

        Motors::setDir(true, true);
        Motors::setDutyCycles(def_drive_to_treasure_duty, def_drive_to_treasure_duty+Motors::default_motors_offset);    // may need to be slower
        Motors::drive();
        
        // go to treasure until in claw range
        double distFrontSonarTreasureClaw = 0;
        do {
            distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonarTreasureClaw);

            ReflectanceSensors::printFrontReflectance();
        } while (distFrontSonarTreasureClaw > treasure_in_claw_dist);
        // Encoders::stopMotorsBrakeEncoders(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Encoders::pulseLW, Encoders::pulseRW, def_drive_to_treasure_duty, 131);
        Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, def_drive_to_treasure_duty, 30);

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