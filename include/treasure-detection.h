#ifndef TreasureDetectionFile
#define TerasureDetectionFile

#include "tape-follower.h"
#include "sonar-sensor.h"

namespace TreasureDetection {
    // initial side sonar detection
    extern const double sideSonarTreasureDists[5]; // cm
    extern const double sideSonarTreasureDistsErr[5]; // cm

    // front sonar detection
    extern const double frontSonarTreasureDists[5]; // cm
    extern const double frontSonarTreasureDistsErr[5]; // cm

    // in claw detection
    extern const double maxTreasureInClawDist; // cm
    
    bool obtainFirstTreasure();
}
#endif
