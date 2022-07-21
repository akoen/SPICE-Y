#include "TapeFollower.h"

// PID tuning
const double kp = 10;
const double ki = 0;
const double kd = 0;
const double maxI = 0;

// vars
bool onTapeL = false;
bool onTapeM = false;
bool onTapeR = false;
bool prevOnTapeL = false;
bool prevOnTapeM = false;
bool prevOnTapeR = false;
int err = 0;
int prevErr = 0;
double pwmChange = 0;  // due to PID
double p = 0;
double d = 0;
double i = 0;

double TapeFollow::calcPidBlackTape() {
    // read reflectance of all 3 sensors (0 or 1 for each)
    ReflectanceSensors::readFrontReflectanceSensors();

    onTapeL = !ReflectanceSensors::frontSensorLval;
    onTapeM = !ReflectanceSensors::frontSensorMval;
    onTapeR = !ReflectanceSensors::frontSensorRval;

    /* truth table: (-) = left, (+) = right
    * M true --> error=0, M R false, L true --> error=1, M R L false, prevL true --> error=2, 
    * M L false, R true --> error=-1, M L R false, prevR true --> error=-2
    */
    if (onTapeM) err = 0;
    else if (!onTapeL && onTapeR) err = -1;
    else if (!onTapeL && !onTapeR && prevOnTapeL) err = -2;
    else if (onTapeL && !onTapeR) err = 1;
    else if (!onTapeL && !onTapeR && prevOnTapeR) err = 2;
    else {}// should have no other states

    p = kp*err, d = kd*(err - prevErr), i += err;

    // anti windup
    if (i > maxI) {
        i = maxI;
    } else if (i < -maxI) {
        i = maxI;
    }
    pwmChange = p + d + i; // > 0 change for right, < 0 for left

    return pwmChange;
}

void TapeFollow::driveWithPid() {
    // get error
    double changePwmSigned = calcPidBlackTape();
    // update pwm
    Motors::setDutyCycles(Motors::dutyCycleL - pwmChange, Motors::dutyCycleR + pwmChange);
    // drive (fwd)
    Motors::setDir(true, true);
    Motors::drive();        
}
