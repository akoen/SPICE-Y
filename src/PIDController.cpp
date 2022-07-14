#include "PIDController.h"
# define REFLECT_LOWER_THRES_BITS 300

// PID
const double kp = 1;
const double ki = 0;
const double kd = 0;
const double maxI = 0;

// PWM drive
const int pwmClockFreq = 100; // hz
const int refDutyCycle = 50; // %

PIDController::PIDController(int sensorL, int sensorM, int sensorR, int pwmMotorL, int pwmMotorR, PinName pwmPinStyleMotorL, PinName pwmPinStyleMotorR): sensorL(sensorL), sensorM(sensorM), sensorR(sensorR),
pwmMotorL(pwmMotorL), pwmMotorR(pwmMotorR), pwmPinStyleMotorL(pwmPinStyleMotorL), pwmPinStyleMotorR(pwmPinStyleMotorR){
    onTapeL = false, onTapeM = false, onTapeR = false;
    prevOnTapeL = onTapeL, prevOnTapeR = onTapeL;
    err = 0;
    prevErr = err;
    pwmChange = 0;

    dutyCycleL = refDutyCycle;
    dutyCycleR = refDutyCycle;

    hasPwmChanged = true;
    configPins();
}

void PIDController::configPins() {
    pinMode(sensorL, INPUT);
    pinMode(sensorM, INPUT);
    pinMode(sensorR, INPUT);

    pinMode(pwmMotorL, OUTPUT);
    pinMode(pwmMotorR, OUTPUT);
}

void PIDController::drive() {
    // init pwm for both wheels
    if (hasPwmChanged) {
        pwm_start(pwmPinStyleMotorL, pwmClockFreq, (int)(dutyCycleL / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorR, pwmClockFreq, (int)(dutyCycleR / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
    }
    
    // compute reflectance from sensors and adjust pwm for wheels respectively
    double pwmChange = compute();

    // adjust
    if (pwmChange != 0) {
        dutyCycleL -= pwmChange;
        dutyCycleR += pwmChange; 
        hasPwmChanged = true;       
    } else {
        hasPwmChanged = false;
    }
    
}

double PIDController::compute() {
    // read reflectance of all 3 sensors (0 or 1 for each)
    double voltBitsL = analogRead(sensorL);
    double voltBitsM = analogRead(sensorM);
    double voltBitsR = analogRead(sensorR);

    onTapeL = voltBitsL > REFLECT_LOWER_THRES_BITS ? false : true;
    onTapeM = voltBitsM > REFLECT_LOWER_THRES_BITS ? false : true;
    onTapeR = voltBitsR > REFLECT_LOWER_THRES_BITS ? false : true;

    /* truth table: (-) = left, (+) = right
     * M true --> error=0, M R false, L true --> error=-1, M R L false, prevL true --> error=-2, 
     * M L false, R true --> error=1, M L R false, prevR true --> error = 2
     */

    if (onTapeM) err = 0;
    else if (!onTapeL && onTapeR) err = -1;
    else if (!onTapeL && !onTapeR && prevOnTapeL) err = -2;
    else if (onTapeL && !onTapeR) err = 1;
    else if (!onTapeL && !onTapeR && prevOnTapeR) err = 2;
    else {}// should have no other states

    p = kp*err, d = kd*(err - prevErr), i += err;

    // anti windup
    if (i > maxI) i = maxI;
    else if (i < - maxI) i = maxI;

    this->pwmChange = p + d + i; // > 0 change for right, < 0 for left

    return this->pwmChange;
}

int PIDController::getErr() {return this->err;}

int PIDController::getDutyCycleL() {return this->dutyCycleL;}
int PIDController::getDutyCycleR() {return this->dutyCycleR;}

void PIDController::setPoint(double setPt) {
    this->setPt = setPt;
}

