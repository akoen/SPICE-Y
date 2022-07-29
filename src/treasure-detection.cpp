#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"

namespace TreasureDetection {
    const double sideSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
    const double sideSonarTreasureDistsErr[5] = {12}; // cm
    const double frontSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
    const double frontSonarTreasureDistsErr[5] = {10}; // cm 
    const double maxTreasureInClawDist = 16; // cm
    const double maxTreasureInClawDistErr = 0; // cm

    bool obtainFirstTreasure() {
        Servos::configArmClawPins();

        double firstSideSonarTreausureDist = sideSonarTreasureDists[0];
        double firstSideSonarTreausureDistErr = sideSonarTreasureDistsErr[0];
        
        double firstFrontSonarTreausureDist = frontSonarTreasureDists[0];
        double firstFrontSonarTreausureDistErr = frontSonarTreasureDistsErr[0];

        // tape follow using PID until first treasure located
        double loopCount = 0;
        double rightSonarDist = 0;
        do {        
            TapeFollow::driveWithPid();
            delay(60);
            rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_R);

            Serial.println("Right sonar dist: ");
            Serial.println(rightSonarDist);

            ReflectanceSensors::printFrontReflectance();

            // if inf loop --> return false
        } while (rightSonarDist > firstSideSonarTreausureDist + firstSideSonarTreausureDistErr || rightSonarDist < firstSideSonarTreausureDist - firstSideSonarTreausureDistErr);
        delay(10);
        Motors::stopMotorsWithBrake(Motors::DRIVE_BACK, LW_PWM_DUTY - 10, 50);
        // Motors::setDir(false, false);
        // Motors::setDutyCycles(LW_PWM_DUTY-10, RW_PWM_DUTY-10);
        // Motors::drive();
        // delay(40);
        // Motors::stopMotors();
        
        Serial.println("found treasure right: ");    
        Serial.println(rightSonarDist);    
        delay(1000);
        
        // turn until front sonar detects treasure
        Encoders::startAddActionCache();

        Motors::rotateRight();
        double distFrontSonar = 0;
        do {
            distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);
            // sonar delay for pulses interference
            delay(60);
            // TODO may need "if fail" handler
        } while(distFrontSonar > firstFrontSonarTreausureDist + firstFrontSonarTreausureDistErr || distFrontSonar < firstFrontSonarTreausureDist - firstFrontSonarTreausureDistErr);   
        delay(1);
        // Motors::stopMotorsWithBrake(Motors::ROTATE_LEFT, Motors::default_rotate_pwm+10, 500);
        Motors::setDir(false, true);
        Motors::setDutyCycles(0, Motors::default_rotate_pwm);
        Motors::drive();
        // Motors::rotateLeft(Motors::default_rotate_pwm+25);
        delay(200);
        Motors::stopMotors();
        
        Encoders::endAddActionCache();

        Serial.println("found treasure front: ");    
        Serial.println(distFrontSonar);
        // drive fwd when front sonar detects treasure
        Encoders::startAddActionCache();

        Motors::setDir(true, true);
        Motors::setDutyCycles(15, 15+Motors::ref_motors_offset);    // may need to be slower
        Motors::drive();
        
        // collect treasure when in range
        double distFrontSonarTreasureClaw = 0;
        do {
            delay(60);
            distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, SONAR_ECHO_PIN_F);

            Serial.print("Front sonar dist: ");
            Serial.println(distFrontSonarTreasureClaw);

            ReflectanceSensors::printFrontReflectance();
        } while (distFrontSonarTreasureClaw > maxTreasureInClawDist);
        Motors::stopMotorsWithBrake(Motors::DRIVE_BACK, 15, 30);

        Encoders::endAddActionCache();
    
        Serial.println("found treasure claw: ");    
        Serial.println(distFrontSonarTreasureClaw);   
        
        // collect
        Servos::collectTreasure();

        // return to original location
        Encoders::executeReverseCache();

        // look for tape
        return TapeFollow::findBlackTape(60);
    }
}
/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */