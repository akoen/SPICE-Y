#include <Arduino.h>
#include "Pins.h"

namespace BombDetection {
    extern bool bombEncounteredFlag;   // only one bomb

    void configMagneticSensorPin();
    
    /**
     * True if sensor detects a magnet (HIGH) 
     */ 
    bool isBombDetected();
}