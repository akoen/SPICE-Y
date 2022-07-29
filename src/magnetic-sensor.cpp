#include "magnetic-sensor.h"

// bool BombDetection::bombEncounteredFlag = false;
namespace BombDetection {
    bool bombEncounteredFlag = true;
    bool hasConfiged = false;

    void configMagneticSensorPin() {
        if (!hasConfiged) pinMode(MAGNETIC_SENSOR_PIN, INPUT);
        hasConfiged = true;
    }

    bool isBombDetected() {
        return !digitalRead(MAGNETIC_SENSOR_PIN);
    }    
}
