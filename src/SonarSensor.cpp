#include "Pins.h"
#include <Arduino.h>


namespace Sonars {
    const double SPEED_SOUND = 0.0343; // cm/s

    void configSonarPins() {
        pinMode(SONAR_TRIG_PIN_L, OUTPUT);
        pinMode(SONAR_ECHO_PIN_L, INPUT);

        pinMode(SONAR_TRIG_PIN_R, OUTPUT);
        pinMode(SONAR_ECHO_PIN_R, INPUT);

        pinMode(SONAR_TRIG_PIN_F, OUTPUT);
        pinMode(SONAR_ECHO_PIN_F, INPUT); 
        
        // clear trig pin
        digitalWrite(SONAR_ECHO_PIN_L, LOW);
        digitalWrite(SONAR_ECHO_PIN_R, LOW);
        digitalWrite(SONAR_ECHO_PIN_F, LOW);
    }
    /**
     * Gets the distance of a single pulse from the specified
     * Note: pulse duration min. 10 microseconds, which is default.
     */
    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration=10) {    
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(trigPin, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        double duration = pulseIn(echoPin, HIGH);
        // Calculating the distance in cm
        return duration * SPEED_SOUND / 2.0;
    }

}