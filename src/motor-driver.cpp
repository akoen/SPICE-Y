#include "motor-driver.h"
#include "reflectance-sensor.h"

namespace Motors {
    int min_drive_dutyCycle = 20;
    int max_drive_dutyCycle = 83;
    const int min_rotate_dutyCycle = 20;

    const int pwm_clock_freq = 100;
    const int def_rotate_pwm = min_rotate_dutyCycle+5;
    const int motors_offset_dutycycle = RW_PWM_DUTY - LW_PWM_DUTY;
    const int def_motors_stop_millis = 200;

    const double WHEELS_WIDTH = 24.5;
    const double WHEEL_DIAMETER = 8.7;

    bool hasPwmChanged = true;
    int dutyCycleL = LW_PWM_DUTY;
    int dutyCycleR = RW_PWM_DUTY;

    bool isLWdirFwd = true;
    bool isRWdirFwd = true;

    std::pair<MotorAction, RotateMode> getInverseDrive(MotorAction motorAction, RotateMode rotateMode) {
        MotorAction inverseAction;
        RotateMode inverseRotate;
        switch(motorAction) {
            case DRIVE_FWD:
                inverseAction = MotorAction::DRIVE_BACK;
                inverseRotate = RotateMode::NONE;
                break;
            case DRIVE_BACK:
                inverseAction = MotorAction::DRIVE_FWD;
                inverseRotate = RotateMode::NONE;
                break;
            case ROTATE_LEFT: 
            case ROTATE_RIGHT:
                if (motorAction == ROTATE_LEFT) inverseAction = MotorAction::ROTATE_RIGHT;
                if (motorAction == ROTATE_RIGHT) inverseAction = MotorAction::ROTATE_LEFT;

                switch(inverseRotate) {
                    case RotateMode::BACKWARDS: 
                        inverseRotate = RotateMode::FORWARDS;
                        break;
                    case RotateMode::FORWARDS: 
                        inverseRotate = RotateMode::BACKWARDS;
                        break;
                    case RotateMode::BOTH_WHEELS: 
                        inverseRotate = RotateMode::BOTH_WHEELS;
                        break;
                    default: 
                        inverseRotate = RotateMode::NONE;
                        break;
                }
                break;
        } 
        return std::pair<MotorAction, RotateMode>(inverseAction, inverseRotate);
    }

    void configMotorPins() {
        pinMode(PWM_MOTOR_FWD_L, OUTPUT);
        pinMode(PWM_MOTOR_FWD_R, OUTPUT);

        pinMode(PWM_MOTOR_BACK_L, OUTPUT);
        pinMode(PWM_MOTOR_BACK_R, OUTPUT);
    }

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

            pwm_start(PWM_FORMAT_MOTOR_FWD_R, pwm_clock_freq, (int)(dutyRfwd / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(PWM_FORMAT_MOTOR_BACK_R, pwm_clock_freq, (int)(dutyRback / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            
            pwm_start(PWM_FORMAT_MOTOR_FWD_L, pwm_clock_freq, (int)(dutyLfwd / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(PWM_FORMAT_MOTOR_BACK_L, pwm_clock_freq, (int)(dutyLback / 100.0 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        
            hasPwmChanged = false;
        }
    }

    void setDutyCycles(int dutyL, int dutyR) {
        if (dutyL < 0) dutyL = 0;
        if (dutyR < 0) dutyR = 0;
        
        // if (dutyL > 0 && dutyL < min_drive_dutyCycle) dutyL = min_drive_dutyCycle;
        // if (dutyR > 0 && dutyR < min_drive_dutyCycle) dutyR = min_drive_dutyCycle;

        if (dutyL > max_drive_dutyCycle) dutyL = max_drive_dutyCycle;
        if (dutyR > max_drive_dutyCycle) dutyR = max_drive_dutyCycle;

        dutyCycleL = dutyL;
        dutyCycleR = dutyR;

        hasPwmChanged = true;
    }

    void setDir(bool isLWdirFwd, bool isRWdirFwd) {
        isLWdirFwd = isLWdirFwd;
        isRWdirFwd = isRWdirFwd;

        hasPwmChanged = true;
    }

    void stopMotorsPWM(int delayMillis) {
        setDir(true, true);
        setDutyCycles(0, 0);
        drive();
        delay(delayMillis);
    }


    void driveFwd(int duty) {
        setDir(true, true);
        setDutyCycles(duty, duty + motors_offset_dutycycle);
        drive();
    }

    void driveBack(int duty) {
        setDir(false, false);
        setDutyCycles(duty, duty + motors_offset_dutycycle);
        drive();
    }

    void rotate(int dutyCycle, bool rotateRight, RotateMode rotateMode) {
        if (dutyCycle < motors_offset_dutycycle) dutyCycle = motors_offset_dutycycle;

        bool _dirLW, _dirRW;
        int dutyLW, dutyRW;

        if (rotateRight) _dirRW = false, _dirLW = true;
        else _dirRW = true, _dirLW = false;
        setDir(_dirLW, _dirRW);

        switch (rotateMode) {
            case BACKWARDS:
                if (rotateRight){
                    setDutyCycles(0, dutyCycle);
                } else {
                    setDutyCycles(dutyCycle, 0);
                }
                break;
            case FORWARDS:
                if (rotateRight) {
                    setDutyCycles(dutyCycle, 0);
                } else {
                    setDutyCycles(0, dutyCycle);
                }
                break;
            case BOTH_WHEELS:
                setDutyCycles(dutyCycle, dutyCycle + motors_offset_dutycycle);
                break;
        }
        drive();
    }

    void stopWithBrake(MotorAction initialAction, RotateMode initialRotateMode, int initialDutyCycle, int durationMillis, int stopMotorsPWMDelayMillis, int offsetDutyRW) {
        // bad input
        if ((initialAction == MotorAction::DRIVE_BACK || initialAction == MotorAction::DRIVE_FWD) && initialRotateMode != RotateMode::NONE) {
            return;
        }
        if ((initialAction == MotorAction::ROTATE_LEFT || initialAction == MotorAction::ROTATE_RIGHT) && initialRotateMode == RotateMode::NONE) {
            return;
        }  
        
        initialDutyCycle += initialDutyCycle *0.15;
        
        bool brakeRotateRight = false;
        RotateMode brakeRotateMode = NONE;

        switch(initialAction) {
            case DRIVE_FWD:
            case DRIVE_BACK:
                if (initialAction == MotorAction::DRIVE_FWD) setDir(false, false);
                else setDir(true, true);
                setDutyCycles(initialDutyCycle, initialDutyCycle+motors_offset_dutycycle);
                drive();
                break;
            case ROTATE_LEFT: 
            case ROTATE_RIGHT:
                if (initialAction == ROTATE_LEFT) brakeRotateRight = true;
                if (initialAction == ROTATE_RIGHT) brakeRotateRight = false;

                if (initialRotateMode == FORWARDS) brakeRotateMode = BACKWARDS;
                else if (initialRotateMode == BACKWARDS) brakeRotateMode = FORWARDS;
                else brakeRotateMode = BOTH_WHEELS;

                rotate(initialDutyCycle, brakeRotateRight, brakeRotateMode);
                break;
        }
        delay(durationMillis);
        stopMotorsPWM(stopMotorsPWMDelayMillis);
    }

    void driveBackRearReflectance(int duty, int stopDuty, int stopMillis) {
        ReflectanceSensors::readSideReflectanceSensors();
        driveBack(duty);
        // both reflectance sensors on surface
        while (!ReflectanceSensors::sideSensorLval && !ReflectanceSensors::sideSensorRval) {
            ReflectanceSensors::readSideReflectanceSensors();
            Serial.print("Reflectance: ");
            Serial.print(ReflectanceSensors::sideSensorLval);
            Serial.print(" ");
            Serial.println(ReflectanceSensors::sideSensorRval);
        }
        // stop w/ greater PWM in case wheels are falling off
        stopWithBrake(MotorAction::DRIVE_BACK, RotateMode::NONE, stopDuty, stopMillis);
    }
}
