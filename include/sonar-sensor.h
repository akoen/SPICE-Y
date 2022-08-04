#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Arduino.h>
#include "config.h"

namespace Sonars {
    extern const double speed_sound; // cm/s
    
    enum SonarType {
        RIGHT,
        LEFT,
        MIDDLE
    };

    void configSonarPins();
    /**
     * Gets the distance of a single pulse from the specified.  delays the program by 60ms to
     * account for interference with previous pulses.
     * Note: pulse duration min. 10 microseconds, which is default.
     */
    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration=10);

    double getAvgDistancePulses(int numReadings, int trigPin, int echoPin, double pulseDuration=10);
}
#endif