#include "Pins.h"
#include <Arduino.h>

// true if HIGH (not reflected), false otherwise
namespace ReflectanceSensors {
    volatile bool frontSensorLval = false;
    volatile bool frontSensorMval = false;
    volatile bool frontSensorRval = false;

    volatile bool sideSensorLval = false;
    volatile bool sideSensorRval = false;

    void configFrontReflectanceSensors() {
        pinMode(SENSOR_PIN_L, INPUT);
        pinMode(SENSOR_PIN_M, INPUT);
        pinMode(SENSOR_PIN_R, INPUT);
    }

    void configSideReflectanceSensors() {
        pinMode(SENSOR_PIN_SIDE_L, INPUT);
        pinMode(SENSOR_PIN_SIDE_R, INPUT);
    }


    void readFrontReflectanceSensors() {
        frontSensorLval = digitalRead(SENSOR_PIN_L);
        frontSensorMval = digitalRead(SENSOR_PIN_M);
        frontSensorRval = digitalRead(SENSOR_PIN_R);
    }

    void readSideReflectanceSensors() {
        sideSensorLval = digitalRead(SENSOR_PIN_SIDE_L);
        sideSensorRval = digitalRead(SENSOR_PIN_SIDE_R);
    }
}

