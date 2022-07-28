#include "magnetic-sensor.h"

// bool BombDetection::bombEncounteredFlag = false;
bool BombDetection::bombEncounteredFlag = true;

void BombDetection::configMagneticSensorPin() {
    pinMode(MAGNETIC_SENSOR_PIN, INPUT);
}

bool BombDetection::isBombDetected() {
    return !digitalRead(MAGNETIC_SENSOR_PIN);
}
