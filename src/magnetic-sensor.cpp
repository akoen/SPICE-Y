#include "magnetic-sensor.h"

namespace BombDetection {
    bool bombEncounteredFlag = false;
    bool hasConfiged = false;

    void configMagneticSensorPin(bool usingInterrupt) {
        if (!hasConfiged) {
            pinMode(MAGNETIC_SENSOR_PIN, INPUT);
            if (usingInterrupt) configInterrupt();
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
