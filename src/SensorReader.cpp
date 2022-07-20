#include "SensorReader.h"
#include <Arduino.h>

SensorReader::SensorReader(int s1, int s2, int s3):sensorL(s1), sensorM(s2), sensorR(s3) {
    pinMode(sensorL, INPUT);
    pinMode(sensorM, INPUT);
    pinMode(sensorR, INPUT);
}

void SensorReader::readSensors() {
    sensorLval = digitalRead(sensorL);
    sensorMval = digitalRead(sensorM);
    sensorRval = digitalRead(sensorR);
}


