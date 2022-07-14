#ifndef PIDControllerFile
#define PIDControllerFile

#include <Arduino.h>

class PIDController {
    public:
        PIDController(int sensorPin1, int sensorPin2, int sensorPin3, int pwmMotorL, int pwmMotorR, 
        PinName pwmPinStyleMotorL, PinName pwmPinStyleMotorR);

        /**
         * @brief Computes the pwm change (%) required to stay on black tape 
         * 
         * @return double - pwm change to stabilize direction, applied to the right if > 0, left if < 0
         */
        double pidBlackTape();
        
        void drive();

        // getters
        int getErr();
        int getDutyCycleL();
        int getDutyCycleR();
    private:
        void configPins();

        // reflectance sensor pins (in)
        int sensorL, sensorM, sensorR;
        // pwm output pins (out)
        int pwmMotorL, pwmMotorR;
        PinName pwmPinStyleMotorL, pwmPinStyleMotorR;

        // duty cycles for each pwm 
        double dutyCycleL, dutyCycleR; // percentage

        // PID control for black tape
        bool onTapeL, onTapeM, onTapeR;
        bool prevOnTapeL, prevOnTapeR;
        int err;
        double p, d, i;
        
        double prevErr;
        double pwmChange;
        bool hasPwmChanged;

        // double setPt, dT, lastT;
};
#endif