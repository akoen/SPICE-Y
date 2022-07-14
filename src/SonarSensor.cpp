#include "SonarSensor.h"

# define SPEED_SOUND 0.0343 // cm/s

SonarSensor::SonarSensor(int trigPin, int echoPin): trigPin(trigPin), echoPin(echoPin) {
    configPins();

    // clear trig pin
    digitalWrite(trigPin, LOW);
}

double SonarSensor::getDistanceSinglePulse(double pulseDuration = 10) {    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    this->duration = pulseIn(echoPin, HIGH);
    // Calculating the distance in cm
    this->distance = duration * SPEED_SOUND / 2.0;
    
    return distance;
}

void SonarSensor::configPins() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}