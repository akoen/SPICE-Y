#ifndef REFLECTANCE_SENSOR
#define REFLECTANCE_SENSOR

#include "config.h"
#include <Arduino.h>

namespace ReflectanceSensors {
    // true if HIGH (not reflected - black tape), false if LOW (reflected - white)
    extern volatile bool frontSensorLval;
    extern volatile bool frontSensorMval;
    extern volatile bool frontSensorRval;
    extern volatile bool sideSensorLval;
    extern volatile bool sideSensorRval;
    /**
     * Configures the front reflectance sensor pins 
     */
    void configFrontReflectanceSensors();
    
    /**
     * Configures the side reflectance sensor pins 
     */
    void configSideReflectanceSensors();
    
    /**
     * Reads and updates the front reflectance sensor values 
     */
    void readFrontReflectanceSensors();
    /**
     * Reads and updates the side reflectance sensor values 
     */
    void readSideReflectanceSensors();
    
    /* testing purposes */
    /**
     * Prints the last-read front reflectance sensor values to serial
     */
    void printFrontReflectance();
}
#endif