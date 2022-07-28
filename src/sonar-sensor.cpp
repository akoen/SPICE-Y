#include "sonar-sensor.h"

namespace Sonars {
    const double speed_sound = 0.0343; // cm/s

    void configSonarPins() {
        pinMode(SONAR_TRIG_PIN_ALL, OUTPUT);
        pinMode(SONAR_ECHO_PIN_L, INPUT);
        pinMode(SONAR_ECHO_PIN_R, INPUT);
        pinMode(SONAR_ECHO_PIN_F, INPUT); 
        
        // clear trig pin
        digitalWrite(SONAR_TRIG_PIN_ALL, LOW);
    }

    double getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration) {    
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(echoPin, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        double duration = pulseIn(echoPin, HIGH);
        // Calculating the distance in cm
        return duration * speed_sound / 2.0;
    }
}
