#ifndef EncoderFile
#define EncoderFile

# include <Arduino.h>

class Encoder {
    public:
        static void ISR_LW();
        static void ISR_RW();
        static void configPins();
        
        void checkPos(int pinL, int pinR);
        // (+): CCW looking from outside, (-): CW looking from outside.
        // i.e.) Right wheel forward = CW, left wheel forward = CCW
};

#endif