#include "Encoder.h"

const double Encoders::pulse_per_rev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
const double Encoders::wheel_diameter = 6.4; // cm

volatile int Encoders::interruptCountLW = 0;
volatile int Encoders::interruptCountRW = 0;
volatile long Encoders::pulseLW = 0;
double Encoders::posLW = 0;
volatile long Encoders::pulseRW = 0;
double Encoders::posRW = 0;

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
