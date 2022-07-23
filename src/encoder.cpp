#include "encoder.h"

const double Encoders::pulse_per_rev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
const double Encoders::wheel_diameter = 6.4; // cm

volatile int Encoders::interruptCountLW = 0;
volatile int Encoders::interruptCountRW = 0;
volatile long Encoders::pulseLW = 0;
double Encoders::posLW = 0;
volatile long Encoders::pulseRW = 0;
double Encoders::posRW = 0;

volatile long cacheStartPulseLW;
volatile long cacheStartPulseRW;

volatile long cacheEndPulseLW;
volatile long cacheEndPulseRW;

std::stack<int>* cachedStackLeftPulses;  
std::stack<int>* cachedStackRightPulses;
// TODO: may add pwm speed for each motor as well

bool cacheCreatedFlag = false;

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

void Encoders::startAddActionCache() {
    if (!cacheCreatedFlag) {
        cachedStackLeftPulses = new std::stack<int>;
        cachedStackRightPulses = new std::stack<int>;

        cacheCreatedFlag = true;
    }

    cacheStartPulseLW = pulseLW;
    cacheStartPulseRW = pulseRW;
}

void Encoders::endAddActionCache() {
    if (!cacheCreatedFlag) {
        // shoulnd't happen - bad
    }

    cacheEndPulseLW = pulseLW;
    cacheEndPulseRW = pulseRW;
    
    cachedStackLeftPulses->push((int)(cacheEndPulseLW - cacheStartPulseLW));
    cachedStackRightPulses->push((int)(cacheEndPulseRW - cacheStartPulseRW));
}

void Encoders::executeReverseCache(int actionDelayMillis) {
    while (!cachedStackLeftPulses->empty() && !cachedStackRightPulses->empty()) {
        delay(actionDelayMillis);

        int actionLeftPulse = cachedStackLeftPulses->top();
        int actionRightPulse = cachedStackRightPulses->top();
        // do stuff

        cachedStackLeftPulses->pop();
        cachedStackRightPulses->pop();
    }

    // clear from heap
    delete cachedStackLeftPulses;
    delete cachedStackRightPulses;

    cachedStackLeftPulses = nullptr;
    cachedStackRightPulses = nullptr;   
}

void Encoders::driveMotorsEncoderPulses(int pulseIntervalLW, int pulseIntervalRW) {
    long startPulseLW = pulseLW;
    long startPulseRW = pulseRW;

    bool dirLW = true, dirRW = true;
    if (pulseIntervalLW < 0) dirLW = false;
    if (pulseIntervalRW < 0) dirRW = false;
    
    Motors::setDir(dirLW, dirRW);
    
}