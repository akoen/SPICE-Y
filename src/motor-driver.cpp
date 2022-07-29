#include "motor-driver.h"


const int Motors::pwm_clock_freq = 100; // hz
const int Motors::ref_duty_cycle = 80; // %
const int Motors::ref_pwm_duty_cycle_LW = LW_PWM_DUTY; // %
const int Motors::ref_pwm_duty_cycle_RW = RW_PWM_DUTY; // %
const int Motors::default_rotate_pwm = 15; // %
const int Motors::ref_motors_offset = Motors::ref_pwm_duty_cycle_RW - Motors::ref_pwm_duty_cycle_LW; // > 0 for RW, < 0 for LW

const double Motors::WHEELS_WIDTH = 24.5;   // cm
const double Motors::WHEEL_DIAMETER = 6.4; // cm


bool Motors::hasPwmChanged = true;    // call pwm start only when changed
int Motors::dutyCycleL = LW_PWM_DUTY;
int Motors::dutyCycleR = RW_PWM_DUTY;

bool Motors::isLWdirFwd = true;
bool Motors::isRWdirFwd = true;

void Motors::configMotorPins() {
    pinMode(PWM_MOTOR_FWD_L, OUTPUT);
    pinMode(PWM_MOTOR_FWD_R, OUTPUT);

    pinMode(PWM_MOTOR_BACK_L, OUTPUT);
    pinMode(PWM_MOTOR_BACK_R, OUTPUT);
}

void Motors::drive() {
    int dutyLfwd = 0;
    int dutyLback = 0;

    int dutyRfwd = 0;
    int dutyRback = 0;

    // init pwm for both wheels
    if (Motors::hasPwmChanged) {
        if (Motors::isLWdirFwd) {
            dutyLfwd = Motors::dutyCycleL;
            dutyLback = 0;
        } else {
            dutyLfwd = 0;
            dutyLback = Motors::dutyCycleL;
        }

        if (Motors::isRWdirFwd) {
            dutyRfwd = Motors::dutyCycleR;
            dutyRback = 0;
        } else {
            dutyRfwd = 0;
            dutyRback = Motors::dutyCycleR; 
        }

        pwm_start(PWM_FORMAT_MOTOR_FWD_R, pwm_clock_freq, (int)(dutyRfwd / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(PWM_FORMAT_MOTOR_BACK_R, pwm_clock_freq, (int)(dutyRback / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        
        pwm_start(PWM_FORMAT_MOTOR_FWD_L, pwm_clock_freq, (int)(dutyLfwd / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(PWM_FORMAT_MOTOR_BACK_L, pwm_clock_freq, (int)(dutyLback / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
    
        Motors::hasPwmChanged = false;
    }
}

void Motors::setDutyCycles(int dutyL, int dutyR) {
    if (dutyL < 0) dutyL = 0;
    if (dutyR < 0) dutyR = 0;

    if (dutyL > 100) dutyL = 100;
    if (dutyR > 100) dutyR = 100;

    Motors::dutyCycleL = dutyL;
    Motors::dutyCycleR = dutyR;

    Motors::hasPwmChanged = true;
}

void Motors::setDir(bool isLWdirFwd, bool isRWdirFwd) {
    Motors::isLWdirFwd = isLWdirFwd;
    Motors::isRWdirFwd = isRWdirFwd;

    hasPwmChanged = true;
}

void Motors::stopMotors(int delayMillis) {
    setDir(true, true);
    setDutyCycles(0, 0);
    drive();
    delay(delayMillis);
}

void Motors::rotateLeft(int dutyCycle, bool bothWheels) {
    if (dutyCycle < ref_motors_offset) {
        dutyCycle = ref_motors_offset;
    }
    if (!bothWheels) setDutyCycles(dutyCycle - ref_motors_offset, 0);  // note: offset best to be an even num
    else setDutyCycles(dutyCycle - ref_motors_offset, dutyCycle);  // note: offset best to be an even num
    setDir(false, true);
    drive();
}

void Motors::rotateRight(int dutyCycle, bool bothWheels) {    
    if (dutyCycle < ref_motors_offset) {
        dutyCycle = ref_motors_offset;
    }
    if (!bothWheels) setDutyCycles(0, dutyCycle+ref_motors_offset);  // note: offset best to be an even num
    else setDutyCycles(dutyCycle, dutyCycle+ref_motors_offset);
    setDir(true, false);
    drive();
}

void Motors::rotate(int dutyCycle, bool rotateRight, bool bothWheels) {
    if (dutyCycle < default_rotate_pwm) dutyCycle = default_rotate_pwm;

    if(rotateRight) {
        setDir(true, false);
        if (!bothWheels) setDutyCycles(0, dutyCycle+ref_motors_offset);  // note: offset best to be an even num
        else setDutyCycles(dutyCycle, dutyCycle+ref_motors_offset);
    } else {
        setDir(false, true);
        if (!bothWheels) setDutyCycles(dutyCycle-ref_motors_offset, 0);  // note: offset best to be an even num
        else setDutyCycles(dutyCycle, dutyCycle+ref_motors_offset);
    }

    drive();
}

void Motors::stopMotorsWithBrake(MotorAction action, int dutyCycle, int durationMillis, bool rotateBothWheels, int stopMotorsDelayMillis) {
    switch(action) {
        case DRIVE_FWD:
            setDutyCycles(dutyCycle, dutyCycle+ref_motors_offset);
            setDir(true, true);
            drive();
        case DRIVE_BACK:
            setDutyCycles(dutyCycle, dutyCycle+ref_motors_offset);
            setDir(false, false);
            drive();
        case ROTATE_LEFT:
            rotateLeft(dutyCycle, rotateBothWheels);
        case ROTATE_RIGHT:
            rotateRight(dutyCycle, rotateBothWheels);
    }
    delay(durationMillis);
    Motors::stopMotors(stopMotorsDelayMillis);
}
/*
 * TODO check that pwm offset is constant across all duty cycles
 */