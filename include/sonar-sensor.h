#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Arduino.h>
#include "config.h"

namespace Sonars {
    extern const double speed_sound; // cm/s
    
    enum SonarType {
        RIGHT = SONAR_ECHO_PIN_R,
        LEFT = SONAR_ECHO_PIN_L,
        FRONT = SONAR_ECHO_PIN_F
    };

    enum PidMode {
        TAPE_FOLLOW,
        IR,
        NONE
    };

    void configSonarPins();
    /**
     * NOTE: No delay added here - delay control must be done external 
     * Gets the distance of a single pulse from the specified. Delays the program by 60ms to
     * account for interference with previous pulses.
     * Note: pulse duration min. 10 microseconds, which is default.
     */
    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration=10, int delayMillis=35);
    
    double getDistanceSinglePulse(Sonars::SonarType sonarType, int delayMillis=35);
    
    /**
     * Gets the avg distance over a certain number of readings
     */
    double getAvgDistancePulses(int numReadings, int trigPin, int echoPin, double pulseDuration=10, int delayMillis=35);

    double getAvgDistancePulses(int numReadings, Sonars::SonarType sonarType, int delayMillis=35);
}

#endif