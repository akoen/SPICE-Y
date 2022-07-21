# include "Pins.h"
# include <Arduino.h>
# include "MotorDriver.cpp"

namespace Encoders {
    const double pulsePerRev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
    const double wheelDiameter = 6.4; // cm

    volatile int interruptCountLW = 0;
    volatile int interruptCountRW = 0;

    volatile long pulseLW = 0;
    volatile double posLW = 0;

    volatile long pulseRW = 0;
    volatile double posRW = 0;

    /**
     * Interrupt service routine (ISR) that will be called each (left) encoder pulse
     */
    void ISR_LW() {
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
    /**
     * Interrupt service routine (ISR) that will be called each (right) encoder pulse
     */
    void ISR_RW() {
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

    void attachInterrupts() {
        attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1), ISR_LW, RISING);
        attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1), ISR_RW, RISING);
    }

    void configEncoderPins() {
        pinMode(L_ENCODER_PIN1, INPUT);
        pinMode(R_ENCODER_PIN1, INPUT);

        Encoders::attachInterrupts();
    }

    void detachEncoderInterrupts() {
        detachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1));
        detachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1));
    }

    void resetEncoderVals() {
        volatile int interruptCountLW = 0;
        volatile int interruptCountRW = 0;

        volatile long pulseLW = 0;
        volatile double posLW = 0;

        volatile long pulseRW = 0;
        volatile double posRW = 0;
    }
}