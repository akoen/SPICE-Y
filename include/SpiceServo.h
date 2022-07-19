#ifndef SERVO_H
#define SERVO_H

#include <Servo.h> //arduino servo library
#include <Arduino.h>

#define CLAW_SERV_PIN PB1
#define ARM_SERV_PIN PB0

#define CLOSE_CLAW 50
#define OPEN_CLAW 100
#define RAISE_ARM 115
#define LOWER_ARM 0

extern Servo *clawServo;
extern Servo *armServo;

void setupServo();
void sendServoSignal();

#endif