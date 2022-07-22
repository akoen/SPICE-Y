#include "TreasureDetector.h"
#include "ServoController.h"

const double TreasureDetection::sideSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::sideSonarTreasureDistsErr[5] = {0.5}; // cm
const double TreasureDetection::frontSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::frontSonarTreasureDistsErr[5] = {0.5}; // cm

const double TreasureDetection::maxTreasureInClawDist = 10; // cm


bool TreasureDetection::obtainFirstTreasure(Adafruit_SSD1306 &display_handler) {

    Servos::configServoPins();
    double firstSideSonarTreausureDist = sideSonarTreasureDists[0];
    double firstSideSonarTreausureDistErr = sideSonarTreasureDistsErr[0];
    
    double firstFrontSonarTreausureDist = frontSonarTreasureDists[0];
    double firstFrontSonarTreausureDistErr = frontSonarTreasureDistsErr[0];
    
    double rightSonarDist = 0;
    do {
        display_handler.clearDisplay();
        display_handler.setCursor(0, 0);
        
        TapeFollow::driveWithPid();
        double rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_R, SONAR_ECHO_PIN_R);
        display_handler.print("Right sonar dist: ");
        display_handler.println(rightSonarDist);
        display_handler.display();
        // if inf loop --> err ret false
    } while (rightSonarDist < firstSideSonarTreausureDist + firstSideSonarTreausureDistErr);
    
    // turn until front sonar detects treasure
    int rotateDutyCycle = 5;
    Motors::rotateLeft(rotateDutyCycle);
    
    double distFrontSonar = 0;
    do {
        display_handler.clearDisplay();
        display_handler.setCursor(0, 0);

        distFrontSonar = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_F, SONAR_ECHO_PIN_F);

        display_handler.print("Front sonar dist: ");
        display_handler.println(distFrontSonar);
        display_handler.display();
        // TODO: need "if fail" handler
    } while(distFrontSonar <= firstFrontSonarTreausureDist 
    + firstFrontSonarTreausureDistErr);   

    delay(1000);
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

/*
 * Two ways of returning to original location after obtaining first treasure:
 * Common: Encoders to drive backwards
 * 1. Rotating to the right until black tape is found --> PID
 * 2. Undo rotation by using encoders
 */