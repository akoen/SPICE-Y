#include "treasure-detection.h"
#include "servo-controller.h"
#include "board-setup.h"
#include "encoder.h"
const double TreasureDetection::sideSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::sideSonarTreasureDistsErr[5] = {2}; // cm
const double TreasureDetection::frontSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::frontSonarTreasureDistsErr[5] = {2}; // cm

const double TreasureDetection::maxTreasureInClawDist = 10; // cm
const double TreasureDetection::maxTreasureInClawDistErr = 0.5; // cm

bool TreasureDetection::obtainFirstTreasure() {
    Servos::configServoPins();

    double firstSideSonarTreausureDist = sideSonarTreasureDists[0];
    double firstSideSonarTreausureDistErr = sideSonarTreasureDistsErr[0];
    
    double firstFrontSonarTreausureDist = frontSonarTreasureDists[0];
    double firstFrontSonarTreausureDistErr = frontSonarTreasureDistsErr[0];

    // tape follow using PID until first treasure located
    double loopCount = 0;
    double rightSonarDist = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);
        
        TapeFollow::driveWithPid();
        rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_R, SONAR_ECHO_PIN_R);

        OLEDDisplayHandler.println("Right sonar dist: ");
        OLEDDisplayHandler.println(rightSonarDist);
        
        OLEDDisplayHandler.println("Loop counter: ");
        OLEDDisplayHandler.println(loopCount++);

        OLEDDisplayHandler.display();
        // if inf loop --> return false
    } while (rightSonarDist > firstSideSonarTreausureDist + firstSideSonarTreausureDistErr || rightSonarDist < firstSideSonarTreausureDist - firstSideSonarTreausureDistErr);
    Motors::stopMotors();
        
    OLEDDisplayHandler.clearDisplay();
    OLEDDisplayHandler.setCursor(0, 0);    
    OLEDDisplayHandler.println("found treasure right: ");    
    OLEDDisplayHandler.println(rightSonarDist);   
    OLEDDisplayHandler.display(); 
    delay(1000);
    
    // turn until front sonar detects treasure
    Encoders::startAddActionCache();

    Motors::rotateLeft();
    double distFrontSonar = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);

        distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);
        // TODO may consider sonar delay for pulses interference
        OLEDDisplayHandler.println("Front sonar dist: ");
        OLEDDisplayHandler.println(distFrontSonar);
        OLEDDisplayHandler.display();
        // TODO may need "if fail" handler
    } while(distFrontSonar > firstFrontSonarTreausureDist + firstFrontSonarTreausureDistErr || 
    distFrontSonar < firstFrontSonarTreausureDist - firstFrontSonarTreausureDistErr);   
    Motors::stopMotors();
    
    Encoders::endAddActionCache();

    OLEDDisplayHandler.clearDisplay();
    OLEDDisplayHandler.setCursor(0, 0);    
    OLEDDisplayHandler.println("found treasure front: ");    
    OLEDDisplayHandler.println(distFrontSonar);
    OLEDDisplayHandler.display();    
    delay(1000);
    
    // drive fwd when front sonar detects treasure
    Encoders::startAddActionCache();

    Motors::setDir(true, true);
    Motors::setDutyCycles(LW_PWM_DUTY, RW_PWM_DUTY);    // may need to be slower
    Motors::drive();
    
    // collect treasure when in range
    double distFrontSonarTreasureClaw = 0;
    do {
        OLEDDisplayHandler.clearDisplay();
        OLEDDisplayHandler.setCursor(0, 0);

        distFrontSonarTreasureClaw = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);

        OLEDDisplayHandler.print("Front sonar dist: ");
        OLEDDisplayHandler.println(distFrontSonarTreasureClaw);
        OLEDDisplayHandler.display();
    } while (distFrontSonarTreasureClaw > maxTreasureInClawDist + maxTreasureInClawDistErr || 
    distFrontSonarTreasureClaw < maxTreasureInClawDist - maxTreasureInClawDistErr);
    Motors::stopMotors();

    Encoders::endAddActionCache();

    OLEDDisplayHandler.clearDisplay();
    OLEDDisplayHandler.setCursor(0, 0);    
    OLEDDisplayHandler.println("found treasure claw: ");    
    OLEDDisplayHandler.println(distFrontSonarTreasureClaw);   
    OLEDDisplayHandler.display();
    
    // collect
    Servos::collectTreasure();

    // return to original location
    Encoders::executeReverseCache();

    return true;
}

/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */