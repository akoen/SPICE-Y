#include "PIDController.h"

# define REFLECT_LOWER_THRES_BITS 300

// PID tuning
const double kp = 1;
const double ki = 0;
const double kd = 0;
const double maxI = 0;

// PWM drive
const int PWM_CLOCK_FREQ = 100; // hz
const int REF_DUTY_CYCLE = 80; // %
const int OFFSET_DUTY_CYCLE = 10; // %

PIDController::PIDController(int sensorPin1, int sensorPin2, int sensorPin3, int pwmMotorFwdL, int pwmMotorBackL, int pwmMotorFwdR, int pwmMotorBackR, 
PinName pwmPinStyleMotorFwdL, PinName pwmPinStyleMotorBackL, 
PinName pwmPinStyleMotorFwdR, PinName pwmPinStyleMotorBackR): sensorL(sensorL), sensorM(sensorM), sensorR(sensorR),pwmMotorFwdL(pwmMotorFwdL), pwmMotorBackL(pwmMotorBackL), pwmMotorFwdR(pwmMotorFwdR), pwmMotorBackR(pwmMotorBackR), pwmPinStyleMotorFwdL(pwmPinStyleMotorFwdL), pwmPinStyleMotorBackL(pwmPinStyleMotorBackL),
pwmPinStyleMotorFwdR(pwmPinStyleMotorFwdR), pwmPinStyleMotorBackR(pwmPinStyleMotorBackR) {
    onTapeL = false, onTapeM = false, onTapeR = false;
    prevOnTapeL = onTapeL, prevOnTapeR = onTapeL;
    err = 0;
    prevErr = err;
    pwmChange = 0;

    dutyCycleL = 84;
    dutyCycleR = 72;
    // dutyCycleL = REF_DUTY_CYCLE;
    // dutyCycleR = REF_DUTY_CYCLE;
    // dutyCycleR = 0;

    hasPwmChanged = true;
    configPins();

    direction = true;   // fwd default

}

void PIDController::configPins() {
    pinMode(sensorL, INPUT);
    pinMode(sensorM, INPUT);
    pinMode(sensorR, INPUT);

    pinMode(this->pwmMotorFwdL, OUTPUT);
    pinMode(this->pwmMotorBackL, OUTPUT);

    pinMode(this->pwmMotorFwdR, OUTPUT);
    pinMode(this->pwmMotorBackR, OUTPUT);
}

void PIDController::updateSensorVals() {
    onTapeL = digitalRead(sensorL);
    onTapeM = digitalRead(sensorM);
    onTapeR = digitalRead(sensorR);

    // onTapeL = onTapeM = onTapeR = false;
}

void PIDController::drive() {
    // init pwm for both wheels
    if (hasPwmChanged) {
        pwm_start(pwmPinStyleMotorFwdL, PWM_CLOCK_FREQ, (int)(dutyCycleL / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorBackL, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

        pwm_start(pwmPinStyleMotorFwdR, PWM_CLOCK_FREQ, (int)(dutyCycleR / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorBackR, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    }
    
    // compute reflectance from sensors and adjust pwm for wheels respectively
    double pwmChange = pidBlackTape();

    // adjust
    if (pwmChange != 0) {
        dutyCycleL -= pwmChange;
        dutyCycleR += pwmChange; 
        hasPwmChanged = true;       
    } else {
        hasPwmChanged = false;
    }
    
}

void PIDController::testDriveNoPID() {
    if (this->hasPwmChanged) {
        pwm_start(pwmPinStyleMotorFwdL, PWM_CLOCK_FREQ, (int)(dutyCycleL / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorBackL, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

        pwm_start(pwmPinStyleMotorFwdR, PWM_CLOCK_FREQ, (int)(dutyCycleR / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorBackR, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        
        this->hasPwmChanged = false;
    }
}
void PIDController::stopPwm() {
        pwm_start(pwmPinStyleMotorBackL, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorFwdL, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

        pwm_start(pwmPinStyleMotorBackR, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorFwdR, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

        this->hasPwmChanged = true;
}  
void PIDController::testBackDriveNoPID() {
    if (this->hasPwmChanged) {
        pwm_start(pwmPinStyleMotorBackL, PWM_CLOCK_FREQ, (int)(dutyCycleL / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorFwdL, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

        pwm_start(pwmPinStyleMotorBackR, PWM_CLOCK_FREQ, (int)(dutyCycleR / 100 * 4096), RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pwmPinStyleMotorFwdR, PWM_CLOCK_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        
        this->hasPwmChanged = false;
    }
}

double PIDController::pidBlackTape() {
    // read reflectance of all 3 sensors (0 or 1 for each)
    // double voltBitsL = analogRead(sensorL);
    // double voltBitsM = analogRead(sensorM);
    // double voltBitsR = analogRead(sensorR);

    // onTapeL = voltBitsL > REFLECT_LOWER_THRES_BITS ? false : true;
    // onTapeM = voltBitsM > REFLECT_LOWER_THRES_BITS ? false : true;
    // onTapeR = voltBitsR > REFLECT_LOWER_THRES_BITS ? false : true;

    updateSensorVals();
    
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
    if (i > maxI) i = maxI;
    else if (i < - maxI) i = maxI;

    this->pwmChange = p + d + i; // > 0 change for right, < 0 for left

    return this->pwmChange;
}

void PIDController::setDutyCycles(int dutyL, int dutyR) {
    this->dutyCycleL = dutyL;
    this->dutyCycleR = dutyR;
}

void PIDController::setHasPwmChanged(bool newPwmStatus) {this->hasPwmChanged = newPwmStatus;} 

int PIDController::getErr() {return this->err;}
int PIDController::getDutyCycleL() {return this->dutyCycleL;}
int PIDController::getDutyCycleR() {return this->dutyCycleR;}
bool PIDController::getHasPwmChanged() {return this->hasPwmChanged;}

bool PIDController::getOnTapeL() {return this->onTapeL;}
bool PIDController::getOnTapeM() {return this->onTapeM;}
bool PIDController::getOnTapeR() {return this->onTapeR;}

