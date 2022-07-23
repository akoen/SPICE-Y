#ifndef MAGNETIC_SENSOR
#define MAGNETIC_SENSOR

#include <Arduino.h>
#include "config.h"

namespace BombDetection {
    extern bool bombEncounteredFlag;   // only one bomb

    void configMagneticSensorPin();
    
    /**
     * True if sensor detects a magnet (HIGH) 
     */ 
    bool isBombDetected();
}
#endif