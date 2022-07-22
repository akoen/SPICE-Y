#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"

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
    
    double rightSonarDist = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);
        
        TapeFollow::driveWithPid();
        double rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_R, SONAR_ECHO_PIN_R);
        OLEDDisplayHandler.print("Right sonar dist: ");
        OLEDDisplayHandler.println(rightSonarDist);
        OLEDDisplayHandler.display();
        // if inf loop --> err ret false
    } while (rightSonarDist > firstSideSonarTreausureDist + firstSideSonarTreausureDistErr);
    Motors::stopMotors();
    delay(1000);
    // turn until front sonar detects treasure
    int rotateDutyCycle = 5;
    Motors::rotateLeft(rotateDutyCycle);
    
    double distFrontSonar = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);

        distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);
        // TODO maybe sonar delay for pulses interference 
        OLEDDisplayHandler.print("Front sonar dist: ");
        OLEDDisplayHandler.println(distFrontSonar);
        OLEDDisplayHandler.display();
        // TODO: need "if fail" handler
    } while(distFrontSonar > firstFrontSonarTreausureDist 
    + firstFrontSonarTreausureDistErr);   
    
    delay(1000);
    // drive fwd when front sonar detects treasure
    Motors::setDir(true, true);
    Motors::setDutyCycles(LW_PWM_DUTY, RW_PWM_DUTY);    // may need to be slower

    // collect treasure when in range
    double distFrontSensorTreasure = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);

        distFrontSensorTreasure = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);

        OLEDDisplayHandler.print("Front sonar dist: ");
        OLEDDisplayHandler.println(distFrontSensorTreasure);
        OLEDDisplayHandler.display();
    } while (distFrontSensorTreasure <= maxTreasureInClawDist);
    Motors::stopMotors();

    // collect
    Servos::collectTreasure();

    return true;
}

/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */