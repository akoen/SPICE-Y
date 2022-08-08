#include "magnetic-sensor.h"

// bool BombDetection::bombEncounteredFlag = false;
namespace BombDetection {
    bool bombEncounteredFlag = false;
    bool hasConfiged = false;

    void configMagneticSensorPin() {
        if (!hasConfiged) {
            pinMode(MAGNETIC_SENSOR_PIN, INPUT);
            configInterrupt();
        }
        hasConfiged = true;
    }
    
    bool isBombDetected() {
        return !digitalRead(MAGNETIC_SENSOR_PIN);
    }

    void configInterrupt() {
        attachInterrupt(digitalPinToInterrupt(MAGNETIC_SENSOR_PIN), BombDetection::ISR_BombDetection, FALLING);
    }    

    void ISR_BombDetection() {
        bombEncounteredFlag = true;
    }
}
