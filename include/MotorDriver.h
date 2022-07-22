#ifndef MotorDriverFile
#define MotorDriverFile

#include <Arduino.h>
#include "Pins.h"

# define LW_PWM_DUTY 84
# define RW_PWM_DUTY 72

namespace Motors {
    extern const int pwm_clock_freq; // hz
    extern const int ref_duty_cycle; // %
    extern const int LW_pwm_duty_cycle; // %
    extern const int RW_pwm_duty_cycle; // %

    extern bool hasPwmChanged;    // call pwm start only when changed
    extern int dutyCycleL;
    extern int dutyCycleR;

    extern bool isLWdirFwd;
    extern bool isRWdirFwd;

    void configMotorPins();
    /**
     * Drives the motors with its duty cycles in the defined direction 
     */ 
    void drive();
    /**
     * Sets the PWM duty cycle for left, right wheels respectively
     * Sets duty cycle = 0 if negative input
     */
    void setDutyCycles(int dutyL, int dutyR);
    /**
     * Sets the direction of the motor to drive.
     * Note that duty cycle is applied to the direction set
     */ 
    void setDir(bool isLWdirFwd, bool isRWdirFwd);
}

#endif