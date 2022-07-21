#include "MotorDriver.h"


const int pwm_clock_freq = 100; // hz
const int ref_duty_cycle = 80; // %
const int LW_pwm_duty_cycle = LW_PWM_DUTY; // %
const int RW_pwm_duty_cycle = RW_PWM_DUTY; // %

bool hasPwmChanged = true;    // call pwm start only when changed
int dutyCycleL = LW_PWM_DUTY;
int dutyCycleR = RW_PWM_DUTY;

bool isLWdirFwd = true;
bool isRWdirFwd = true;

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
    if (hasPwmChanged) {
        if (isLWdirFwd) {
            dutyLfwd = dutyCycleL;
            dutyLback = 0;
        } else {
            dutyLfwd = 0;
            dutyLback = dutyCycleL;
        }

        if (isRWdirFwd) {
            dutyRfwd = dutyCycleR;
            dutyRback = 0;
        } else {
            dutyRfwd = 0;
            dutyRback = dutyCycleR; 
        }

        pwm_start(PWM_FORMAT_MOTOR_FWD_R, pwm_clock_freq, (int)(dutyRfwd / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(PWM_FORMAT_MOTOR_BACK_R, pwm_clock_freq, (int)(dutyRback / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        
        pwm_start(PWM_FORMAT_MOTOR_FWD_L, pwm_clock_freq, (int)(dutyLfwd / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(PWM_FORMAT_MOTOR_BACK_L, pwm_clock_freq, (int)(dutyLback / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
    
        hasPwmChanged = false;
    }
}

void Motors::setDutyCycles(int dutyL, int dutyR) {
    if (dutyL < 0) dutyL = 0;
    if (dutyR < 0) dutyR = 0;

    dutyCycleL = dutyL;
    dutyCycleR = dutyR;

    hasPwmChanged = true;
}

void Motors::setDir(bool isLWdirFwd, bool isRWdirFwd) {
    Motors::isLWdirFwd = isLWdirFwd;
    Motors::isRWdirFwd = isRWdirFwd;
}
