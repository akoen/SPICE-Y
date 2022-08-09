#ifndef MAGNETIC_SENSOR
#define MAGNETIC_SENSOR

#include <Arduino.h>
#include "config.h"

namespace BombDetection {
    extern bool bombEncounteredFlag;   // only one bomb
    extern bool hasConfiged;
    /**
     * Configures the magnetic sensor pin once and attaches an interrupt to the pin.
     */
    void configMagneticSensorPin(bool usingInterrupt);
    
    /**
     * True if sensor detects a magnet (LOW) 
     */ 
    bool isBombDetected();

    /**
     * Attaches the magnetic sensor pin to be used with interrupts, to detemrine if a bomb has been detected when falling from
     * HIGH to LOW.
     */ 
    void configInterrupt();

    /**
     * Interrupt service routine for bomb detection. To be used when the magnetic bomb has been detected, which pulls the pin from HIGH to LOW.
     */  
    void ISR_BombDetection();

}
#endif