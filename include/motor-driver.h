#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include <Arduino.h>
#include "config.h"
#include <utility>


# define LW_PWM_DUTY 76
# define RW_PWM_DUTY 72

namespace Motors {
    extern const int pwm_clock_freq; // hz
    extern const int def_rotate_pwm; // %

    extern const int motors_offset_dutycycle; // %, > 0 for RW, < 0 for LW
    extern const int def_motors_stop_millis; // %, > 0 for RW, < 0 for LW

    extern int max_drive_dutyCycle;
    extern int min_drive_dutyCycle;
    extern const int min_rotate_dutyCycle;

    extern const double WHEELS_WIDTH;
    extern const double WHEEL_DIAMETER;

    extern bool hasPwmChanged; // call pwm start only when changed
    extern int dutyCycleL;
    extern int dutyCycleR;

    extern bool isLWdirFwd;
    extern bool isRWdirFwd;

    /**
     * Represents the different actions that can be taken
     */
    enum MotorAction {
        DRIVE_FWD,
        DRIVE_BACK, 
        ROTATE_LEFT, 
        ROTATE_RIGHT  
    };

    /**
     * Represents the different types of rotations that can be taken
     */
    enum RotateMode {
        BACKWARDS,
        FORWARDS,
        BOTH_WHEELS,
        NONE
    };

    /**
     * Gets the inverse given the motor action and rotation type.
     * 
     * Driving fwd <-> driving back
     * Forwards rotation to the left <-> backwards rotation to the right
     * Forwards rotation to the right <-> backwards rotation to the left 
     */
    std::pair<MotorAction, RotateMode> getInverseDrive(MotorAction motorAction, RotateMode rotateMode);

    /**
     * Configures all motor pins 
     */
    void configMotorPins();
    
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
     * Note that duty cycle is applied to the specified direction
     */ 
    void setDir(bool isLWdirFwd, bool isRWdirFwd);

    /**
     * Drives the motors with its duty cycles in the defined direction 
     */ 
    void drive();

    /**
     * Stops the motors. Halts the program for the specified duration (ms) to account
     * for motor inertia. 
     */
    void stopMotorsPWM(int delayMillis=def_motors_stop_millis);

    /**
     * Stops the motors by applying the opposite action at a specifc pwm
     * duty cycle for a specified duration of the driving action.
     */
    void stopWithBrake(MotorAction initialAction, RotateMode initialRotateMode, int initialDutyCycle, int durationMillis, int stopMotorsPWMDelayMillis=def_motors_stop_millis, int offsetDutyRW=0);
    
    /**
     * Drives the motors forwards for the specified duty cycle. Handles any motor offset
     */
    void driveFwd(int duty);

    /**
     * Drives the motors backwards for the specified duty cycle. Handle any motor offset
     */
    void driveBack(int duty);
    
    /**
     * Rotates motor with the given pwm duty cycle
     * Accounts for ref duty cycle offset and the given duty cycle saturates
     * if magnitude is larger than motors offset 
     */
    void rotate(int dutyCycle, bool rotateRight, RotateMode mode);
    
    /**
     * Drives the motors backwards with the specified duty cycle as long as both rear reflectance sensors are reading HIGH. 
     */
    void driveBackRearReflectance(int duty, int stopDuty, int stopMillis);
}

#endif