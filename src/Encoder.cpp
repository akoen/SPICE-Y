# include "Encoder.h"

Encoder::Encoder(int encoderPinL, int encoderPinR): pinA(encoderPinL), pinB(encoderPinR) {
    this->interruptCount = 0;
    this->revs = 0;
    this->dir = true;
    this->pos = 0; 
}

void Encoder::ISR() {
    checkPos(pinA, pinB);
}

/**
 * @brief Obtains the position of the motor. To be only called by the ISR.
 * 
 * @param pinA encoder pin A
 * @param pinB encoder pin B
 */
void Encoder::checkPos(int pinA, int pinB) {
    int encoderAState = digitalRead(pinA);
    int encoderBState = digitalRead(pinB);

    if (encoderAState == 0) {
        // shouldn't occur - rising edge
        this->revs = -1;
    }
    if (encoderBState == 0) {  // CW (my def)
        this->revs--;
        this->dir = false;
    } else {    // CCW
        this->revs++;
        this->dir = true;
    } 
    interruptCount++;
}

void Encoder::configPins() {
    pinMode(this->pinA, INPUT);
    pinMode(this->pinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(this->pinA), ISR, RISING);

}

