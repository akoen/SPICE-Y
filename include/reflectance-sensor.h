#ifndef ReflectanceSensorFile
#define ReflectanceSensorFile

#include "Pins.h"
#include <Arduino.h>

namespace ReflectanceSensors {
    // true if HIGH (not reflected - black tape), false if LOW (reflected - white)
    extern volatile bool frontSensorLval;
    extern volatile bool frontSensorMval;
    extern volatile bool frontSensorRval;
    extern volatile bool sideSensorLval;
    extern volatile bool sideSensorRval;

    void configFrontReflectanceSensors();
    void configSideReflectanceSensors();
    
    void readFrontReflectanceSensors();
    void readSideReflectanceSensors();
}
#endif