#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include <Arduino.h>
#include "config.h"
#include <utility>

// # define LW_PWM_DUTY 84 - 45
// # define RW_PWM_DUTY 72 - 45
# define LW_PWM_DUTY 42
# define RW_PWM_DUTY 50
# define CHICKEN_WIRE_OFFSET_DUTY 14

namespace Motors {
    extern const int pwm_clock_freq; // hz
    extern const int ref_duty_cycle; // %
    extern const int ref_pwm_duty_cycle_LW; // %
    extern const int ref_pwm_duty_cycle_RW; // %

    extern const int default_rotate_pwm; // %

    extern const int default_motors_offset; // %, > 0 for RW, < 0 for LW
    extern const int default_motors_stop_millis; // %, > 0 for RW, < 0 for LW

    extern const int min_drive_dutyCycle;
    extern const int min_rotate_dutyCycle;

    extern const double WHEELS_WIDTH;
    extern const double WHEEL_DIAMETER;

    extern bool hasPwmChanged; // call pwm start only when changed
    extern int dutyCycleL;
    extern int dutyCycleR;

    extern bool isLWdirFwd;
    extern bool isRWdirFwd;

    enum MotorAction {
        DRIVE_FWD,
        DRIVE_BACK, 
        ROTATE_LEFT, 
        ROTATE_RIGHT  
    };

    enum RotateMode {
        BACKWARDS,
        FORWARDS,
        BOTH_WHEELS,
        NONE
    };

    std::pair<MotorAction, RotateMode> getInverseDrive(MotorAction motorAction, RotateMode rotateMode);

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
    void stopMotorsPWM(int delayMillis=default_motors_stop_millis);
                
    /**
     * Rotates motor left with the given pwm duty cycle.
     * Accounts for ref duty cycle offset and the given duty cycle saturates if magnitude is larger than motors offset 

     */
    void rotateLeft(int dutyCycle=default_rotate_pwm, bool bothWheels=false);
    /**
     * Rotates motor right with the given pwm duty cycle
     * Accounts for ref duty cycle offset and the given duty cycle saturates
     * if magnitude is larger than motors offset 
     */
    void rotateRight(int dutyCycle=default_rotate_pwm, bool bothWheels=false);

    
    void rotate(int dutyCycle, bool rotateRight, RotateMode mode);

    /**
     * Stops the motors by applying the opposite action at a specifc pwm
     * duty cycle for a specified duration of the driving action.
     */
    void stopWithBrake(MotorAction initialAction, RotateMode initialRotateMode, int initialDutyCycle, int durationMillis, int stopMotorsPWMDelayMillis=default_motors_stop_millis);
}

#endif