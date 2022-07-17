// # include "Encoder.h"

// # define ENCODER_LW_A PB0
// # define ENCODER_LW_B PB1

// # define ENCODER_RW_A PB2
// # define ENCODER_RW_B PB3

// Encoder::Encoder() {
//     this->interruptCount = 0;
//     this->revs = 0;
//     this->dir = true;
//     this->pos = 0; 
// }

// void ISR_LW() {
//     int encoderAState = digitalRead(ENCODER_LW_A);
//     int encoderBState = digitalRead(ENCODER_LW_B);

//     if (encoderAState == 0) {
//         // shouldn't occur - rising edge
//         this->revs = -1;
//     }
//     if (encoderBState == 0) {  // CW (my def)
//         this->revs--;
//         this->dir = false;
//     } else {    // CCW
//         this->revs++;
//         this->dir = true;
//     } 
//     interruptCount++;
// }

// void ISR_RW() {

// }

// /**
//  * @brief Obtains the position of the motor. To be only called by the ISR.
//  * 
//  * @param pinA encoder pin A
//  * @param pinB encoder pin B
//  */
//  void Encoder::checkPos(int pinA, int pinB) {
//     int encoderAState = digitalRead(pinA);
//     int encoderBState = digitalRead(pinB);

//     if (encoderAState == 0) {
//         // shouldn't occur - rising edge
//         this->revs = -1;
//     }
//     if (encoderBState == 0) {  // CW (my def)
//         this->revs--;
//         this->dir = false;
//     } else {    // CCW
//         this->revs++;
//         this->dir = true;
//     } 
//     interruptCount++;
// }

// void Encoder::configPins() {
//     pinMode(this->pinA, INPUT);
//     pinMode(this->pinB, INPUT);

//     attachInterrupt(digitalPinToInterrupt(pinA), ISR, RISING);

// }

