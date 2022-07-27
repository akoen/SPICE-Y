#include "encoder.h"

// const double Encoders::pulse_per_rev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
const double Encoders::pulse_per_rev = 131*11 / 10.0;  // divide by counter at end, increases pulse width
const double Encoders::wheel_diameter = 6.4; // cm

volatile int Encoders::interruptCountLW = 0;
volatile int Encoders::interruptCountRW = 0;
volatile long Encoders::pulseLW = 0;
double Encoders::posLW = 0;
volatile long Encoders::pulseRW = 0;
double Encoders::posRW = 0;

volatile long Encoders::cacheStartPulseLW;
volatile long Encoders::cacheStartPulseRW;

volatile long Encoders::cacheEndPulseLW;
volatile long Encoders::cacheEndPulseRW;

std::stack<int>* Encoders::cachedActionsLeftPulses;  
std::stack<int>* Encoders::cachedActionsRightPulses;
// TODO: may add pwm speed for each motor as well

bool Encoders::cacheCreated = false;
bool Encoders::cacheAddInProgress = false;

void Encoders::ISR_LW() {
    int state1 = digitalRead(L_ENCODER_PIN1);
    if (state1 == 0) {
        pulseLW = -99999; // shouldn't occur
    } else {    // CCW
        if (Motors::isLWdirFwd) {
            pulseLW++;
        } else {
            pulseLW--;
        }
    } 
    interruptCountLW++;
}

void Encoders::ISR_RW() {
    int state1 = digitalRead(R_ENCODER_PIN1);
    if (state1 == 0) {
        pulseRW = -99999; // shouldn't occur
    } else {    // CCW
        if (Motors::isRWdirFwd) {
            pulseRW++;
        } else {
            pulseRW--;
        }
    } 
    interruptCountRW++;
}

void Encoders::attachInterrupts() {
    attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1), Encoders::ISR_LW, RISING);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1), Encoders::ISR_RW, RISING);
}

void Encoders::configEncoderPins() {
    pinMode(L_ENCODER_PIN1, INPUT);
    pinMode(R_ENCODER_PIN1, INPUT);

    Encoders::attachInterrupts();
}

void Encoders::detachEncoderInterrupts() {
    detachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1));
    detachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1));
}

void Encoders::resetEncoderVals() {
    Encoders::interruptCountLW = 0;
    Encoders::interruptCountRW = 0;

    Encoders::pulseLW = 0;
    Encoders::posLW = 0;

    Encoders::pulseRW = 0;
    Encoders::posRW = 0;
}

bool Encoders::startAddActionCache() {
    if (cacheAddInProgress) return false;
    
    if (!cacheCreated) {
        cachedActionsLeftPulses = new std::stack<int>;
        cachedActionsRightPulses = new std::stack<int>;

        cacheCreated = true;
    } 
    cacheStartPulseLW = pulseLW;
    cacheStartPulseRW = pulseRW;

    cacheAddInProgress = true;

    return true;
}

bool Encoders::endAddActionCache() {
    if (!cacheCreated || !cacheAddInProgress) return false;

    cacheEndPulseLW = pulseLW;
    cacheEndPulseRW = pulseRW;    
    cachedActionsLeftPulses->push((int)(cacheEndPulseLW - cacheStartPulseLW));
    cachedActionsRightPulses->push((int)(cacheEndPulseRW - cacheStartPulseRW));
    
    cacheAddInProgress = false;

    return true;
}

bool Encoders::executeReverseCache(int actionDelayMillis) {
    if (!cacheCreated || cacheAddInProgress) return false;

    while (!cachedActionsLeftPulses->empty() && !cachedActionsRightPulses->empty()) {        
        // execute cached actions in reverse
        int actionLeftPulse = -1 * cachedActionsLeftPulses->top();
        int actionRightPulse = -1 * cachedActionsRightPulses->top();
        cachedActionsLeftPulses->pop();
        cachedActionsRightPulses->pop();

        driveMotorsEncoderPulses(actionLeftPulse, actionRightPulse);

        delay(actionDelayMillis);
    }

    // clear from heap
    delete cachedActionsLeftPulses;
    delete cachedActionsRightPulses;

    cachedActionsLeftPulses = nullptr;
    cachedActionsRightPulses = nullptr; 

    cacheCreated = false;
    
    return true;  
}

void Encoders::driveMotorsEncoderPulses(int pulseIntervalLW, int pulseIntervalRW) {
    long startPulseLW = pulseLW;
    long startPulseRW = pulseRW;

    bool dirLW = true, dirRW = true;
    if (pulseIntervalLW < 0) dirLW = false;
    if (pulseIntervalRW < 0) dirRW = false;
    
    long startEncoderPulsesLW = pulseLW;
    long startEncoderPulsesRW = pulseRW;

    if (dirLW && !dirRW) {  // rotate right
        Motors::rotateRight();  // TODO may cache pwm
    } else if (!dirLW && dirRW) {   // rotate left
        Motors::rotateLeft();
    } else {   // fwd or backwards 
        Motors::setDir(dirLW, dirRW);
        Motors::setDutyCycles(LW_PWM_DUTY, RW_PWM_DUTY);    // TODO may cache pwm
        Motors::drive();
    }

    while (pulseLW < startEncoderPulsesLW + pulseIntervalLW && pulseRW < pulseIntervalRW) {
        /* pulse LW,RW vals should be updated by interrupts */
    }
    Motors::stopMotors();
}