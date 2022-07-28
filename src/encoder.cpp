#include "encoder.h"

namespace Encoders {
    // const double Encoders::pulse_per_rev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
    const double pulse_per_rev = 131*11 / 10.0;  // divide by counter at end, increases pulse width

    volatile int interruptCountLW = 0;
    volatile int interruptCountRW = 0;
    volatile long pulseLW = 0;
    double posLW = 0;
    volatile long pulseRW = 0;
    double posRW = 0;

    volatile long cacheStartPulseLW;
    volatile long cacheStartPulseRW;

    volatile long cacheEndPulseLW;
    volatile long cacheEndPulseRW;

    std::stack<int>* cachedActionsLeftPulses;  
    std::stack<int>* cachedActionsRightPulses;
    // TODO: may add pwm speed for each motor as well

    bool cacheCreated = false;
    bool cacheAddInProgress = false;

    void ISR_LW() {
        int state1 = digitalRead(L_ENCODER_PIN1);
        if (state1 == 0) {
            // pulseLW = -99999; 
            // shouldn't occur
        } else {    // CCW
            if (Motors::isLWdirFwd) {
                pulseLW++;
            } else {
                pulseLW--;
            }
        } 
        interruptCountLW++;
    }

    void ISR_RW() {
        int state1 = digitalRead(R_ENCODER_PIN1);
        if (state1 == 0) {
            // pulseRW = -99999; 
            // shouldn't occur
        } else {    // CCW
            if (Motors::isRWdirFwd) {
                pulseRW++;
            } else {
                pulseRW--;
            }
        } 
        interruptCountRW++;
    }

    void attachInterrupts() {
        attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1), Encoders::ISR_LW, RISING);
        attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1), Encoders::ISR_RW, RISING);
    }

    void configEncoderPins() {
        pinMode(L_ENCODER_PIN1, INPUT);
        pinMode(R_ENCODER_PIN1, INPUT);

        attachInterrupts();
    }

    void detachEncoderInterrupts() {
        detachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1));
        detachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1));
    }

    void resetEncoderVals() {
        interruptCountLW = 0;
        interruptCountRW = 0;

        pulseLW = 0;
        posLW = 0;

        pulseRW = 0;
        posRW = 0;
    }

    bool startAddActionCache() {
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

    bool endAddActionCache() {
        if (!cacheCreated || !cacheAddInProgress) return false;

        cacheEndPulseLW = pulseLW;
        cacheEndPulseRW = pulseRW;    
        cachedActionsLeftPulses->push((int)(cacheEndPulseLW - cacheStartPulseLW));
        cachedActionsRightPulses->push((int)(cacheEndPulseRW - cacheStartPulseRW));
        
        cacheAddInProgress = false;

        return true;
    }

    bool executeReverseCache(int actionDelayMillis) {
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

    void driveMotorsEncoderPulses(int pulseIntervalLW, int pulseIntervalRW) {
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
        while (pulseLW < startEncoderPulsesLW + pulseIntervalLW && pulseRW < startEncoderPulsesRW + pulseIntervalRW) {
            /* pulse LW,RW vals should be updated by interrupts */
        }
        Motors::stopMotors();
    }

    void driveMotorsDistance(bool dirFwd, double distance) {
        if (!dirFwd) distance *= -1;

        // convert to pulses - distance per pulse = pi*diameter / pulse per rev
        double distPerPulse = PI * Motors::WHEEL_DIAMETER / pulse_per_rev;  // cm
        int pulsesInterval = round(distance/distPerPulse);

        driveMotorsEncoderPulses(pulsesInterval, pulsesInterval);
    }

    void rotateMotorsDegs(bool dirRight, double angle) {
        double anglePerPulse = 180 * (PI*Motors::WHEEL_DIAMETER / pulse_per_rev) / Motors::WHEELS_WIDTH;
        int pulses = round(angle / anglePerPulse);
        if (dirRight) driveMotorsEncoderPulses(0, pulses);
        else driveMotorsEncoderPulses(pulses, 0);
    }

}