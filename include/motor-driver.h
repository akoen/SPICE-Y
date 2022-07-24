#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include <Arduino.h>
#include "config.h"

# define LW_PWM_DUTY 84
# define RW_PWM_DUTY 72
# define CHICKEN_WIRE_OFFSET_DUTY 7

namespace Motors {
    extern const int pwm_clock_freq; // hz
    extern const int ref_duty_cycle; // %
    extern const int ref_pwm_duty_cycle_LW; // %
    extern const int ref_pwm_duty_cycle_RW; // %
    extern const int default_rotate_pwm; // %
    extern const int ref_motors_offset; // %, > 0 for RW, < 0 for LW

    extern bool hasPwmChanged; // call pwm start only when changed
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
     * 
     * Recommended to use this function for changing duty cycles instead of directly
     * as it updates that pwm has been changed
     */
    void setDutyCycles(int dutyL, int dutyR);
    /**
     * Sets the direction of the motor to drive.
     * Note that duty cycle is applied to the direction set
     */ 
    void setDir(bool isLWdirFwd, bool isRWdirFwd);

    /**
     * Stops the motors. Halts the program for the specified duration (ms) to account
     * for motor inertia. 
     */
    void stopMotors(int delayMillis=1000);

    /**
     * Rotates motor left with the given pwm duty cycle.
     * Accounts for ref duty cycle offset and the given duty cycle saturates
     * if magnitude is larger than motors offset 
     */
    void rotateLeft(int dutyCycle=default_rotate_pwm);
    /**
     * Rotates motor right with the given pwm duty cycle
     * Accounts for ref duty cycle offset and the given duty cycle saturates
     * if magnitude is larger than motors offset 
     */
    void rotateRight(int dutyCycle=default_rotate_pwm);
}

#endif