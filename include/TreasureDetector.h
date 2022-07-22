#ifndef TreasureDetectionFile
#define TerasureDetectionFile

#include "TapeFollower.h"
#include "SonarSensor.h"
#include <Adafruit_SSD1306.h>

namespace TreasureDetection {
    // initial side sonar detection
    extern const double sideSonarTreasureDists[5]; // cm
    extern const double sideSonarTreasureDistsErr[5]; // cm

    // front sonar detection
    extern const double frontSonarTreasureDists[5]; // cm
    extern const double frontSonarTreasureDistsErr[5]; // cm

    // in claw detection
    extern const double maxTreasureInClawDist; // cm

    bool obtainFirstTreasure(Adafruit_SSD1306 &display_handler);
}
#endif
