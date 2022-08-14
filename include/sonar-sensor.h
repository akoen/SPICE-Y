#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Arduino.h>
#include "config.h"

namespace Sonars {
    extern const double speed_sound; // cm/us
    
    /**
     * Represents the different sonars that can be used.
     */
    enum SonarType {
        RIGHT = SONAR_ECHO_PIN_R,
        LEFT = SONAR_ECHO_PIN_L,
        FRONT = SONAR_ECHO_PIN_F
    };

    /**
     * Configures all sonar trigger and echo pins.
     */
    void configSonarPins();
    
    /**
     * Gets the distance of a single pulse from the specified. Delays the program by the specified amount to
     * account for interference with previous pulses. If no delay has been added, must be controlled externally.
     * pulse duration min. 10 microseconds, which is default.
     */
    double getDistanceSinglePulse(Sonars::SonarType sonarType, int delayMillis=35);
    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration=10, int delayMillis=35);
    
    /**
     * Gets the avg distance over a certain number of readings
     */
    double getAvgDistancePulses(int numReadings, Sonars::SonarType sonarType, int delayMillis=35);
    double getAvgDistancePulses(int numReadings, int trigPin, int echoPin, double pulseDuration=10, int delayMillis=35);
}

#endif