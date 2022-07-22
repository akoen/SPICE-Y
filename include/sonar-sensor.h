#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Arduino.h>
#include "config.h"

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