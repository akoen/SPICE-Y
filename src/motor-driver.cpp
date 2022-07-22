#include "motor-driver.h"


const int Motors::pwm_clock_freq = 100; // hz
const int Motors::ref_duty_cycle = 80; // %
const int Motors::LW_pwm_duty_cycle = LW_PWM_DUTY; // %
const int Motors::RW_pwm_duty_cycle = RW_PWM_DUTY; // %

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

void Motors::stopMotors() {
    setDir(true, true);
    setDutyCycles(0, 0);
    drive();
}

void Motors::rotateLeft(int dutyCycle) {
    isLWdirFwd = true;
    isRWdirFwd = false;
    
    int motorsOffset = RW_PWM_DUTY - LW_PWM_DUTY;
    if (dutyCycle < motorsOffset || dutyCycle > motorsOffset) {
        dutyCycle = motorsOffset;
    }
    setDutyCycles(dutyCycle - (motorsOffset / 2), dutyCycle + (motorsOffset / 2));  // note: offset best to be an even num
    drive();
}

void Motors::rotateRight(int dutyCycle) {
    isLWdirFwd = false;
    isRWdirFwd = true;
    
    int motorsOffset = RW_PWM_DUTY - LW_PWM_DUTY;
    if (dutyCycle < motorsOffset || dutyCycle > motorsOffset) {
        dutyCycle = motorsOffset;
    }
    setDutyCycles(dutyCycle - (motorsOffset / 2), dutyCycle + (motorsOffset / 2));  // note: offset best to be an even num
    drive();
}

