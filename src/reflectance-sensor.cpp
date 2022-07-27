#include "reflectance-sensor.h"

namespace ReflectanceSensors {
    volatile bool frontSensorLval = false;
    volatile bool frontSensorMval = false;
    volatile bool frontSensorRval = false;
    volatile bool sideSensorLval = false;
    volatile bool sideSensorRval = false;

    void configFrontReflectanceSensors() {
        pinMode(REFLECTANCE_PIN_L, INPUT);
        pinMode(REFLECTANCE_PIN_M, INPUT);
        pinMode(REFLECTANCE_PIN_R, INPUT);
    }
    void configSideReflectanceSensors() {
        pinMode(REFLECTANCE_PIN_SIDE_L, INPUT);
        pinMode(REFLECTANCE_PIN_SIDE_R, INPUT);
    }
    void readFrontReflectanceSensors() {
        // HIGH (1) if not reflected, LOW (0) if reflected
        frontSensorLval = digitalRead(REFLECTANCE_PIN_L);
        frontSensorMval = digitalRead(REFLECTANCE_PIN_M);
        frontSensorRval = digitalRead(REFLECTANCE_PIN_R);
    }  
    void readSideReflectanceSensors() {
        sideSensorLval = digitalRead(REFLECTANCE_PIN_SIDE_L);
        sideSensorRval = digitalRead(REFLECTANCE_PIN_SIDE_R);
    }
    /* testing purposes */
    void printFrontReflectance() {
        Serial.print("Sensors(L,M,R): ");
        Serial.print(frontSensorLval);
        Serial.print(" ");
        Serial.print(frontSensorMval);
        Serial.print(" ");
        Serial.println(frontSensorRval);
    }
}