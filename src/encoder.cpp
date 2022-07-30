#include "encoder.h"
sdsdffsdsf
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

            // get modes

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

    void driveMotorsEncoderPulses(int dutyCycle, Motors::MotorAction motorAction, Motors::RotateMode rotateMode, int pulseInterval) {
        // bad input
        if ((motorAction == Motors::MotorAction::DRIVE_BACK || motorAction == Motors::MotorAction::DRIVE_FWD) && rotateMode != Motors::RotateMode::NONE) {
            return;
        }
        if ((motorAction == Motors::MotorAction::ROTATE_LEFT || motorAction == Motors::MotorAction::ROTATE_RIGHT) && rotateMode == Motors::RotateMode::NONE) {
            return;
        }
        if (pulseInterval < 0) return;

        long startPulseLW = pulseLW;
        long startPulseRW = pulseRW;

        switch (motorAction) {
            case Motors::FORWARDS:
                Motors::setDir(true, true);
                Motors::setDutyCycles(dutyCycle, dutyCycle);
                Motors::drive();
                while (pulseLW < startPulseLW + pulseInterval && pulseRW < startPulseRW + pulseInterval);
            case Motors::BACKWARDS:
                Motors::setDir(false, false);
                Motors::setDutyCycles(dutyCycle, dutyCycle);
                Motors::drive();
                while (pulseLW > startPulseLW - pulseInterval && pulseRW > startPulseRW - pulseInterval);
            case Motors::ROTATE_LEFT:
            case Motors::ROTATE_RIGHT:
                switch(rotateMode) {
                    case Motors::BACKWARDS:
                        if (motorAction == Motors::ROTATE_LEFT) {
                            Motors::rotate(dutyCycle, false, rotateMode);
                            while (pulseLW > startPulseLW - pulseInterval);
                        } else {
                            Motors::rotate(dutyCycle, true, rotateMode);
                            while (pulseRW > startPulseRW - pulseInterval);
                        }
                    case Motors::FORWARDS:
                        if (motorAction == Motors::ROTATE_LEFT) {
                            Motors::rotate(dutyCycle, false, rotateMode);
                            while (pulseLW > startPulseLW + pulseInterval);
                        } else {
                            Motors::rotate(dutyCycle, true, rotateMode);
                            while (pulseRW > startPulseRW + pulseInterval);
                        }
                    case Motors::BOTH_WHEELS:
                        if (motorAction == Motors::ROTATE_LEFT) {
                            Motors::rotate(dutyCycle, false, rotateMode);
                            while (pulseLW > startPulseLW - pulseInterval && pulseRW < startPulseRW + pulseInterval);
                        } else {
                            Motors::rotate(dutyCycle, true, rotateMode);
                            while (pulseLW > startPulseLW + pulseInterval && pulseRW > startPulseRW - pulseInterval);
                        }
                    case Motors::NONE:
                        // bad input
                }
        }
    }

    void driveMotorsDistance(int dutyCycle, bool dirFwd, double distance) {
        // convert to pulses - distance per pulse = pi*diameter / pulse per rev
        double distPerPulse = PI * Motors::WHEEL_DIAMETER / pulse_per_rev;  // cm
        int pulsesInterval = round(distance/distPerPulse);

        Motors::MotorAction driveAction = dirFwd ? Motors::MotorAction::DRIVE_FWD : Motors::MotorAction::DRIVE_BACK;
        driveMotorsEncoderPulses(dutyCycle, driveAction, Motors::RotateMode::NONE, pulsesInterval);
    }

    void rotateMotorsDegs(int dutyCycle, bool dirRight, Motors::RotateMode rotateMode, double angle) {
        double anglePerPulse = 180 * (PI*Motors::WHEEL_DIAMETER / pulse_per_rev) / Motors::WHEELS_WIDTH;
        
        int pulses = round(angle / anglePerPulse);
        Motors::MotorAction driveAction = dirRight ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT;
        driveMotorsEncoderPulses(dutyCycle, driveAction, rotateMode, pulses);
    }
}