#include "treasure-detection.h"
#include "servo-controller.h"

const double TreasureDetection::sideSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::sideSonarTreasureDistsErr[5] = {0.5}; // cm
const double TreasureDetection::frontSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::frontSonarTreasureDistsErr[5] = {0.5}; // cm

const double TreasureDetection::maxTreasureInClawDist = 10; // cm

bool TreasureDetection::obtainFirstTreasure() {

    Servos::configServoPins();
    double firstSideSonarTreausureDist = sideSonarTreasureDists[0];
    double firstSideSonarTreausureDistErr = sideSonarTreasureDistsErr[0];
    
    double firstFrontSonarTreausureDist = frontSonarTreasureDists[0];
    double firstFrontSonarTreausureDistErr = frontSonarTreasureDistsErr[0];
    
    TapeFollow::driveWithPid();
    
    double rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_R, SONAR_ECHO_PIN_R);
    
    // treasure found
    if (rightSonarDist < firstSideSonarTreausureDist + firstSideSonarTreausureDistErr) {
        // run routine:
        
        // turn until front sonar detects treasure
        int rotateDutyCycle = 5;
        Motors::rotateLeft(rotateDutyCycle);

        double distFrontSonar = 0;
        do {
            distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);
            // TODO: need "if fail" handler
        } while(distFrontSonar <= firstFrontSonarTreausureDist 
        + firstFrontSonarTreausureDistErr);    
        
        // drive fwd when front sonar detects treasure
        Motors::setDir(true, true);
        Motors::setDutyCycles(LW_PWM_DUTY, RW_PWM_DUTY);    // may need to be slower

        // collect treasure when in range
        double distFrontSensorTreasure = 0;
        do {
            distFrontSensorTreasure = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);
        } while (distFrontSensorTreasure <= maxTreasureInClawDist);
        Motors::stopMotors();

        // collect
        Servos::collectTreasure();

        return true;
    }
    
    // shouldn't occur - did not obtain treasure
    return false;
}

/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */