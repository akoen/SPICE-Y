#include "sonar-sensor.h"

const double Sonars::speed_sound = 0.0343; // cm/s

void Sonars::configSonarPins() {
    pinMode(SONAR_TRIG_PIN_L, OUTPUT);
    pinMode(SONAR_ECHO_PIN_L, INPUT);

    pinMode(SONAR_TRIG_PIN_R, OUTPUT);
    pinMode(SONAR_ECHO_PIN_R, INPUT);

    pinMode(SONAR_TRIG_PIN_F, OUTPUT);
    pinMode(SONAR_ECHO_PIN_F, INPUT); 
    
    // clear trig pin
    digitalWrite(SONAR_TRIG_PIN_L, LOW);
    digitalWrite(SONAR_TRIG_PIN_R, LOW);
    digitalWrite(SONAR_TRIG_PIN_F, LOW);
}

double Sonars::getDistanceSinglePulse(int trigPin, int echoPin, double pulseDuration) {    
    digitalWrite(SONAR_TRIG_PIN_R, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(SONAR_TRIG_PIN_R, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(SONAR_ECHO_PIN_R, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    double duration = pulseIn(SONAR_ECHO_PIN_R, HIGH);
    // Calculating the distance in cm
    return duration * speed_sound / 2.0;
}
