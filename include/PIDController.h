#ifndef PIDControllerFile
#define PIDControllerFile

#include <Arduino.h>

class PIDController {
    public:
        PIDController(int sensorPin1, int sensorPin2, int sensorPin3, int pwmMotorL, int pwmMotorR, 
        PinName pwmPinStyleMotorL, PinName pwmPinStyleMotorR);

        void setPoint(double setPoint);
        double compute();
        void drive();

        // getters
        int getErr();
        int getDutyCycleL();
        int getDutyCycleR();
    private:
        void configPins();

        int sensorL, sensorM, sensorR;
        int pwmMotorL, pwmMotorR;

        PinName pwmPinStyleMotorL, pwmPinStyleMotorR;

        double dutyCycleL, dutyCycleR; // percentage
        bool onTapeL, onTapeM, onTapeR;
        bool prevOnTapeL, prevOnTapeR;
        double setPt; 
        int err;
        double p, d, i;
        // double dT, lastT;
        double prevErr;
        double pwmChange;
        bool hasPwmChanged;
};
#endif