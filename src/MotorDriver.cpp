# include "Pins.h"
#include <Arduino.h>

namespace Motors {
    // PWM drive
    const int PWM_CLOCK_FREQ = 100; // hz
    const int REF_DUTY_CYCLE = 80; // %
    const int LW_PWM_DUTY_CYCLE = 84; // %
    const int RW_PWM_DUTY_CYCLE = 72; // %

    volatile bool hasPwmChanged = true;    // call pwm start only when changed
    volatile int dutyCycleL = LW_PWM_DUTY_CYCLE;
    volatile int dutyCycleR = RW_PWM_DUTY_CYCLE;

    volatile bool isLWdirFwd = true;
    volatile bool isRWdirFwd = true;
    
    void configMotorPins() {
        pinMode(PWM_MOTOR_FWD_L, OUTPUT);
        pinMode(PWM_MOTOR_FWD_R, OUTPUT);

        pinMode(PWM_MOTOR_BACK_L, OUTPUT);
        pinMode(PWM_MOTOR_BACK_R, OUTPUT);
    }

    /**
     * Drives the motors with its duty cycles in the defined direction 
     */ 
    void drive() {
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

            pwm_start(PWM_FORMAT_MOTOR_FWD_R, PWM_CLOCK_FREQ, (int)(dutyRfwd / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(PWM_FORMAT_MOTOR_BACK_R, PWM_CLOCK_FREQ, (int)(dutyRback / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            
            pwm_start(PWM_FORMAT_MOTOR_FWD_L, PWM_CLOCK_FREQ, (int)(dutyLfwd / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(PWM_FORMAT_MOTOR_BACK_L, PWM_CLOCK_FREQ, (int)(dutyLback / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        
            hasPwmChanged = false;
        }
    }
    
    /**
     * Sets the PWM duty cycle for left, right wheels respectively
     * Sets duty cycle = 0 if negative input
     */
    void setDutyCycles(int dutyL, int dutyR) {
        if (dutyL < 0) dutyL = 0;
        if (dutyR < 0) dutyR = 0;

        dutyCycleL = dutyL;
        dutyCycleR = dutyR;

        hasPwmChanged = true;
    }

    /**
     * Sets the direction of the motor to drive.
     * Note that duty cycle is applied to the direction set
     */ 
    void setDir(bool isLWdirfwd, bool isRWdirfwd) {
        Motors::isLWdirFwd = isLWdirFwd;
        Motors::isRWdirFwd = isRWdirFwd;
    }

}