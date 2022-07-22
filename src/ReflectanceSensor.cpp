#include "ReflectanceSensor.h"

volatile bool ReflectanceSensors::frontSensorLval = false;
volatile bool ReflectanceSensors::frontSensorMval = false;
volatile bool ReflectanceSensors::frontSensorRval = false;
volatile bool ReflectanceSensors::sideSensorLval = false;
volatile bool ReflectanceSensors::sideSensorRval = false;

void ReflectanceSensors::configFrontReflectanceSensors() {
    pinMode(REFLECTANCE_PIN_L, INPUT);
    pinMode(REFLECTANCE_PIN_M, INPUT);
    pinMode(REFLECTANCE_PIN_R, INPUT);
}
void ReflectanceSensors::configSideReflectanceSensors() {
    pinMode(REFLECTANCE_PIN_SIDE_L, INPUT);
    pinMode(REFLECTANCE_PIN_SIDE_R, INPUT);
}
void ReflectanceSensors::readFrontReflectanceSensors() {
    // HIGH (1) if not reflected, LOW (0) if reflected
    ReflectanceSensors::frontSensorLval = digitalRead(REFLECTANCE_PIN_L);
    ReflectanceSensors::frontSensorMval = digitalRead(REFLECTANCE_PIN_M);
    ReflectanceSensors::frontSensorRval = digitalRead(REFLECTANCE_PIN_R);
}  
void ReflectanceSensors::readSideReflectanceSensors() {
    ReflectanceSensors::sideSensorLval = digitalRead(REFLECTANCE_PIN_SIDE_L);
    ReflectanceSensors::sideSensorRval = digitalRead(REFLECTANCE_PIN_SIDE_R);
}
