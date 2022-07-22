#include "TreasureDetector.h"

const double TreasureDetection::sideSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::sideSonarTreasureDistsErr[5] = {0.5}; // cm
const double TreasureDetection::frontSonarTreasureDists[5] = {20, 20, 20, 20, 20}; // cm
const double TreasureDetection::frontSonarTreasureDistsErr[5] = {0.5}; // cm

const double TreasureDetection::maxTreasureInClawDist = 10; // cm

bool TreasureDetection::obtainFirstTreasure() {
    double firstSideSonarTreausureDist = sideSonarTreasureDists[0];
    double firstSideSonarTreausureDistErr = sideSonarTreasureDistsErr[0];
    
    TapeFollow::driveWithPid();

    double rightSonarDist = Sonars::getDistanceSinglePulse(SONAR_TRIG_PIN_R, SONAR_ECHO_PIN_R);
    
    // treasure found
    if (rightSonarDist < firstSideSonarTreausureDist + firstSideSonarTreausureDistErr 
    && rightSonarDist > firstSideSonarTreausureDist - firstSideSonarTreausureDistErr) {
        // run routine:

        // turn until front sonar detects treasure 
        // drive fwd when front sonar detects treasure
        // collect treasure
        
        return true;
    }
    
    // shouldn't occur - did not obtain treasure
    return false;
}
