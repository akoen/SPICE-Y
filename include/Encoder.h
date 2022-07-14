#ifndef EncoderFile
#define EncoderFile

# include <Arduino.h>

class Encoder {
    public:
        Encoder(int pinL, int pinR);
        void ISR();

    private:
        void configPins();
        double checkPos(int pinL, int pinR);
        
        int pinL, pinR;
        u_int32_t interruptCount;
        u_int32_t revs;
        // (+): CCW looking from outside, (-): CW looking from outside.
        // i.e.) Right wheel forward = CW, left wheel forward = CCW
        bool direction;
        double pos;
};

#endif