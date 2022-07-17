// #ifndef EncoderFile
// #define EncoderFile

// # include <Arduino.h>

// void ISR_LW();
// void ISR_RW();

// class Encoder {
//     public:
//         Encoder();

//     private:
//         void configPins();
//         void checkPos(int pinL, int pinR);
        
//         int pinA, pinB;
//         u_int32_t interruptCount;
//         u_int32_t revs;
//         // (+): CCW looking from outside, (-): CW looking from outside.
//         // i.e.) Right wheel forward = CW, left wheel forward = CCW
//         bool dir;
//         double pos;
// };

// #endif