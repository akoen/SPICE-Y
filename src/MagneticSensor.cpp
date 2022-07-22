#include "MagneticSensor.h"

bool BombDetection::bombEncounteredFlag = false;

void BombDetection::configMagneticSensorPin() {
    pinMode(MAGNETIC_SENSOR_PIN, INPUT);
}

bool BombDetection::isBombDetected() {
    return digitalRead(MAGNETIC_SENSOR_PIN);
}