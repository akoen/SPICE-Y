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

    volatile long cacheStartPulse;
    volatile long cacheEndPulse;

    std::stack<std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>*>* cachedActions;     
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

    bool startAddActionCache(Motors::MotorAction motorAction, Motors::RotateMode rotateMode, int dutyCycle) {
        // bad input
        if ((motorAction == Motors::MotorAction::DRIVE_BACK || motorAction == Motors::MotorAction::DRIVE_FWD) && rotateMode != Motors::RotateMode::NONE) {
            return false;
        }
        if ((motorAction == Motors::MotorAction::ROTATE_LEFT || motorAction == Motors::MotorAction::ROTATE_RIGHT) && rotateMode == Motors::RotateMode::NONE) {
            return false;
        }
        if (cacheAddInProgress) return false;

        if (!cacheCreated) {
            cachedActions = new std::stack<std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>*>();
            cacheCreated = true;
        } 
        std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>* tuplePtr = 
        new std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>(motorAction, rotateMode, dutyCycle, 0);
        cachedActions->push(tuplePtr);
        
        switch (motorAction) {
            case Motors::MotorAction::DRIVE_FWD:
            case Motors::MotorAction::DRIVE_BACK:
                cacheStartPulse = pulseRW;  // pulses travelled for LW RW should be same (ideally - if wheels rotate at same speed at given duty cycle)
            case Motors::MotorAction::ROTATE_LEFT:
            case Motors::MotorAction::ROTATE_RIGHT:
                switch(rotateMode) {
                    case Motors::RotateMode::BACKWARDS:
                        if (motorAction == Motors::MotorAction::ROTATE_LEFT) {
                            // LW moves
                            cacheStartPulse = pulseLW;
                        } else {
                            // RW moves 
                            cacheStartPulse = pulseRW; 
                        }
                    case Motors::RotateMode::FORWARDS:
                        if (motorAction == Motors::MotorAction::ROTATE_LEFT) {
                            // RW moves
                            cacheStartPulse = pulseRW;
                        } else {
                            // LW moves 
                            cacheStartPulse = pulseLW; 
                        }
                    case Motors::RotateMode::BOTH_WHEELS:
                        // LW, RW moves - ideally same for given duty cycle
                        cacheStartPulse = pulseRW;
                }
        }
        cacheAddInProgress = true;

        return true;
    }

    bool endAddActionCache() {
        if (!cacheCreated || !cacheAddInProgress) return false;
        Motors::MotorAction motorAction = std::get<0>(*cachedActions->top());
        Motors::RotateMode rotateMode = std::get<1>(*cachedActions->top());;
        
        switch (motorAction) {
            case Motors::MotorAction::DRIVE_FWD:
            case Motors::MotorAction::DRIVE_BACK:
                cacheEndPulse = pulseRW;  // pulses travelled for LW RW should be same (ideally - if wheels rotate at same speed at given duty cycle)
            case Motors::MotorAction::ROTATE_LEFT:
            case Motors::MotorAction::ROTATE_RIGHT:
                switch(rotateMode) {
                    case Motors::RotateMode::BACKWARDS:
                        if (motorAction == Motors::MotorAction::ROTATE_LEFT) {
                            // LW moves
                            cacheEndPulse = pulseLW;
                        } else {
                            // RW moves 
                            cacheEndPulse = pulseRW; 
                        }
                    case Motors::RotateMode::FORWARDS:
                        if (motorAction == Motors::MotorAction::ROTATE_LEFT) {
                            // RW moves
                            cacheEndPulse = pulseRW;
                        } else {
                            // LW moves 
                            cacheEndPulse = pulseLW; 
                        }
                    case Motors::RotateMode::BOTH_WHEELS:
                        // LW, RW moves - ideally same for given duty cycle
                        cacheEndPulse = pulseRW;
                }
        }
        std::get<3>(*cachedActions->top()) = cacheEndPulse - cacheStartPulse;

        cacheAddInProgress = false;

        return true;
    }

    bool executeReverseCache(int actionDelayMillis) {
        if (!cacheCreated || cacheAddInProgress) return false;

        while (!cachedActions->empty()) {        
            // // execute cached actions in reverse

            Motors::MotorAction motorAction;
            Motors::RotateMode rotateMode;
            int dutyCycle, pulseInterval;
            
            std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>* topCachedAction = cachedActions->top();
            std::tie(motorAction, rotateMode, dutyCycle, pulseInterval) = *topCachedAction;
            
            Motors::MotorAction inverseAction;
            Motors::RotateMode inverseRotate;

            // get inverse action
            std::tie(inverseAction, inverseRotate) = Motors::getInverseDrive(motorAction, rotateMode);
            
            int inversePulseInterval = -pulseInterval;

            // pop from stack (and delete from heap)
            cachedActions->pop();
            
            delete topCachedAction;
            topCachedAction = nullptr;

            // execute             
            driveMotorsEncoderPulses(dutyCycle, inverseAction, inverseRotate, inversePulseInterval);

            // stop
            delay(10);
            Motors::stopWithBrake(inverseAction, inverseRotate, dutyCycle, 10, actionDelayMillis);
        }
    
        // clear entire cache from heap
        delete cachedActions;
        cachedActions = nullptr;

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
                        break;
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