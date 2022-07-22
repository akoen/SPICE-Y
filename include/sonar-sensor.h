#ifndef SonarSensorFile
#define SonarSensorFile

#include <Arduino.h>
#include "Pins.h"

namespace Sonars {
    extern const double speed_sound; // cm/s

    void configSonarPins();
    /**
     * Gets the distance of a single pulse from the specified
     * Note: pulse duration min. 10 microseconds, which is default.
     */
    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration=10);
}
#endif