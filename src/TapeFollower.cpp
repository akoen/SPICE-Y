#include "TapeFollower.h"

// PID tuning
const double TapeFollow::kp = 20;
const double TapeFollow::ki = 0;
const double TapeFollow::kd = 10;
const double TapeFollow::maxI = 0;

// vars
bool TapeFollow::onTapeL = false;
bool TapeFollow::onTapeM = false;
bool TapeFollow::onTapeR = false;
bool TapeFollow::prevOnTapeL = false;
bool TapeFollow::prevOnTapeM = false;
bool TapeFollow::prevOnTapeR = false;

int TapeFollow::err = 0;
int TapeFollow::prevErr = 0;
double TapeFollow::pwmChange = 0;  // due to PID
double TapeFollow::p = 0;
double TapeFollow::d = 0;
double TapeFollow::i = 0;

bool TapeFollow::startFlag = false;
long TapeFollow::prevErrTime = 0;
long TapeFollow::currTime = 0;

double TapeFollow::calcPidBlackTape() {
    // if (!startFlag) {
    //     prevErrTime = millis();
    //     currTime = prevErrTime;
    // } 
    // read reflectance of all 3 sensors (0 or 1 for each)
    ReflectanceSensors::readFrontReflectanceSensors();

    onTapeL = ReflectanceSensors::frontSensorLval;
    onTapeM = ReflectanceSensors::frontSensorMval;
    onTapeR = ReflectanceSensors::frontSensorRval;

    /* truth table: (-) = left, (+) = right
    * M true --> error=0, M R false, L true --> error=1, M R L false, prevL true --> error=2, 
    * M L false, R true --> error=-1, M L R false, prevR true --> error=-2
    */
    if (onTapeL && onTapeM && onTapeR) {
        // Motors::setDutyCycles(LW_PWM_DUTY-CHICKEN_WIRE_OFFSET_DUTY, RW_PWM_DUTY+CHICKEN_WIRE_OFFSET_DUTY);    // chicken wire
        Motors::dutyCycleL = LW_PWM_DUTY - CHICKEN_WIRE_OFFSET_DUTY;
        Motors::dutyCycleL = RW_PWM_DUTY + CHICKEN_WIRE_OFFSET_DUTY;
        return 0;
    }
    if (onTapeL && onTapeM) err = -1;
    else if (onTapeM && onTapeR) err = 1;
    else if (onTapeM) err = 0;
    else if (!onTapeL && onTapeR) err = -2;
    else if (!onTapeL && !onTapeR && prevOnTapeL) err = 5;
    else if (onTapeL && !onTapeR) err = 2;
    else if (!onTapeL && !onTapeR && prevOnTapeR) err = -5;
    else {
        // L M R all off tape, L R previously off tape -- do nothing
    }// should have no other states

    p = kp*err;
    currTime = millis();
    if (startFlag && err != 0) {
        // d = kd*(err - prevErr) / (1.0*(currTime - prevErrTime));   
        d = kd*(err - prevErr);   
        prevErrTime = currTime;     
    } else {
        d = 0;
        startFlag = true;
    }
    i += err;

    // anti windup
    if (i > maxI) {
        i = maxI;
    } else if (i < -maxI) {
        i = maxI;
    }
    pwmChange = p + d + i; // > 0 change for right, < 0 for left    // TODO: bad code - make local

    prevOnTapeL = onTapeL;
    prevOnTapeR = onTapeR;
    prevErr = err;
    return pwmChange;
}

void TapeFollow::driveWithPid() {
    // get error
    double changePwmSigned = calcPidBlackTape();
    // update pwm
    Motors::setDutyCycles(Motors::dutyCycleL - changePwmSigned, Motors::dutyCycleR + changePwmSigned);
    // if (changePwmSigned > 0) {
    //     Motors::setDutyCycles(Motors::dutyCycleL-pwmChange, Motors::dutyCycleR);
    // } else {
    //     Motors::setDutyCycles(Motors::dutyCycleL, Motors::dutyCycleR+pwmChange);
    // }
    // drive (fwd)
    Motors::setDir(true, true);
    Motors::drive();        
}
