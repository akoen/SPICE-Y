#include "motor-driver.h"


const int Motors::pwm_clock_freq = 100; // hz
const int Motors::ref_duty_cycle = 80; // %
const int Motors::ref_pwm_duty_cycle_LW = LW_PWM_DUTY; // %
const int Motors::ref_pwm_duty_cycle_RW = RW_PWM_DUTY; // %
const int Motors::default_rotate_pwm = 8; // %
const int Motors::default_motors_offset = Motors::ref_pwm_duty_cycle_RW - Motors::ref_pwm_duty_cycle_LW; // > 0 for RW, < 0 for LW
const int Motors::default_motors_stop_millis = 500;

const double Motors::WHEELS_WIDTH = 24.5;   // cm
const double Motors::WHEEL_DIAMETER = 6.4; // cm


bool Motors::hasPwmChanged = true;    // call pwm start only when changed
int Motors::dutyCycleL = LW_PWM_DUTY;
int Motors::dutyCycleR = RW_PWM_DUTY;

bool Motors::isLWdirFwd = true;
bool Motors::isRWdirFwd = true;

std::pair<Motors::MotorAction, Motors::RotateMode> Motors::getInverseDrive(MotorAction motorAction, RotateMode rotateMode) {
    MotorAction inverseAction;
    RotateMode inverseRotate;
    switch(motorAction) {
        case DRIVE_FWD:
            inverseAction = MotorAction::DRIVE_BACK;
            inverseRotate = RotateMode::NONE;
        case DRIVE_BACK:
            inverseAction = MotorAction::DRIVE_FWD;
            inverseRotate = RotateMode::NONE;
        case ROTATE_LEFT: 
        case ROTATE_RIGHT:
            if (motorAction == ROTATE_LEFT) inverseAction = MotorAction::ROTATE_RIGHT;
            if (motorAction == ROTATE_RIGHT) inverseAction = MotorAction::ROTATE_LEFT;

            switch(inverseRotate) {
                case RotateMode::BACKWARDS: inverseRotate = RotateMode::FORWARDS;
                case RotateMode::FORWARDS: inverseRotate = RotateMode::BACKWARDS;
                case RotateMode::BOTH_WHEELS: inverseRotate = RotateMode::BOTH_WHEELS;
                default: inverseRotate = RotateMode::NONE;
            }
    } 
    return std::pair<MotorAction, RotateMode>(inverseAction, inverseRotate);
}

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
    if (dutyCycle < default_motors_offset) {
        dutyCycle = default_motors_offset;
    }
    if (!bothWheels) setDutyCycles(dutyCycle, 0);  // note: offset best to be an even num
    else setDutyCycles(dutyCycle, dutyCycle+ref_duty_cycle);  // note: offset best to be an even num
    setDir(false, true);
    drive();
}

void Motors::rotateRight(int dutyCycle, bool bothWheels) {    
    if (dutyCycle < default_motors_offset) {
        dutyCycle = default_motors_offset;
    }
    if (!bothWheels) setDutyCycles(0, dutyCycle+default_motors_offset);  // note: offset best to be an even num
    else setDutyCycles(dutyCycle, dutyCycle+default_motors_offset);
    setDir(true, false);
    drive();
}

void Motors::rotate(int dutyCycle, bool rotateRight, RotateMode rotateMode) {
    if (dutyCycle < default_motors_offset) dutyCycle = default_motors_offset;

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
            setDutyCycles(dutyCycle, dutyCycle + default_motors_offset);
            break;
    }
    drive();
}

void Motors::stopWithBrake(MotorAction initialAction, RotateMode initialRotateMode, int initialDutyCycle, int durationMillis, int stopMotorsDelayMillis) {
    // bad input
    if ((initialAction == MotorAction::DRIVE_BACK || initialAction == MotorAction::DRIVE_FWD) && initialRotateMode != RotateMode::NONE) {
        return;
    }
    if ((initialAction == MotorAction::ROTATE_LEFT || initialAction == MotorAction::ROTATE_RIGHT) && initialRotateMode == RotateMode::NONE) {
        return;
    }  
    bool brakeRotateRight = false;
    RotateMode brakeRotateMode = NONE;

    switch(initialAction) {
        case DRIVE_FWD:
            setDutyCycles(initialDutyCycle, initialDutyCycle+default_motors_offset);
            setDir(false, false);
            drive();
        case DRIVE_BACK:
            setDutyCycles(initialDutyCycle, initialDutyCycle+default_motors_offset);
            setDir(true, true);
            drive();
        case ROTATE_LEFT: 
        case ROTATE_RIGHT:
            if (initialAction == ROTATE_LEFT) brakeRotateRight = true;
            if (initialAction == ROTATE_LEFT) brakeRotateRight = false;

            if (initialRotateMode == FORWARDS) brakeRotateMode = BACKWARDS;
            else if (initialRotateMode == BACKWARDS) brakeRotateMode = FORWARDS;
            else brakeRotateMode = BOTH_WHEELS;

            rotate(initialDutyCycle, brakeRotateRight, brakeRotateMode);
    }
    delay(durationMillis);
    Motors::stopMotors(stopMotorsDelayMillis);
}
/*
 * TODO check that pwm offset is constant across all duty cycles
 */