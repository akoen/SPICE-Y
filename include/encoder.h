#ifndef ENCODER
#define ENCODER

#include "config.h"
#include "motor-driver.h"
#include <Arduino.h>

namespace Encoders {
    extern const double pulse_per_rev;  // divide by counter at end, increases pulse width
    extern const double wheel_diameter; // cm

    extern volatile int interruptCountLW;
    extern volatile int interruptCountRW;

    extern volatile long pulseLW;
    extern double posLW;
    extern volatile long pulseRW;
    extern double posRW;
    
    /**
     * Interrupt service routine (ISR) that will be called each (left) encoder pulse
     */
    void ISR_LW();
    /**
     * Interrupt service routine (ISR) that will be called each (right) encoder pulse
     */
    void ISR_RW();

    void attachInterrupts();
    void configEncoderPins();
    void detachEncoderInterrupts();
    void resetEncoderVals();
}
#endif