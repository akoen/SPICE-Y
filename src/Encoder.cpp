#include "Encoder.h"

const double pulse_per_rev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
const double wheel_diameter = 6.4; // cm

int interruptCountLW = 0;
int interruptCountRW = 0;
long pulseLW = 0;
double posLW = 0;
long pulseRW = 0;
double posRW = 0;

void Encoders::ISR_LW() {
    int state1 = digitalRead(L_ENCODER_PIN1);
    if (state1 == 0) {
        pulseLW = -99999; // shouldn't occur
    } else {    // CCW
        if(Motors::isLWdirFwd) {
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

    attachInterrupts();
}

void Encoders::detachEncoderInterrupts() {
    detachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1));
    detachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1));
}

void Encoders::resetEncoderVals() {
    int interruptCountLW = 0;
    int interruptCountRW = 0;

    long pulseLW = 0;
    double posLW = 0;

    long pulseRW = 0;
    double posRW = 0;
}
