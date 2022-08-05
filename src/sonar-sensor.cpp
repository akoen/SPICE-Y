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
        // sonar delay for pulse interference
        delay(60);
        // clear trig pin 
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(echoPin, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        double duration = pulseIn(echoPin, HIGH, 500000UL);
        // Calculating the distance in cm
        return duration * speed_sound / 2.0;
    }

    double getDistanceSinglePulse(Sonars::SonarType sonarType) {
        return getDistanceSinglePulse(SONAR_TRIG_PIN_ALL, sonarType);
    }

    double getAvgDistancePulses(int numReadings, int trigPin, int echoPin, double pulseDuration) {
        double avgSonarDist = 0;
        for (int i = 0; i < numReadings; i++) {
            avgSonarDist += Sonars::getDistanceSinglePulse(trigPin, echoPin);
        }
        return avgSonarDist / (1.0 * numReadings);
    }

    double getAvgDistancePulses(int numReadings, Sonars::SonarType sonarType) {
        return getAvgDistancePulses(numReadings, SONAR_TRIG_PIN_ALL, sonarType);
    }

}
