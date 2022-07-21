// #ifndef PIDControllerFile
// #define PIDControllerFile

// #include <Arduino.h>
// #include <Adafruit_SSD1306.h>

// class PIDController {
//     public:
//         PIDController(int sensorPin1, int sensorPin2, int sensorPin3, int pwmMotorFwdL, int pwmMotorBackL, int pwmMotorFwdR, 
//         int pwmMotorBackR, PinName pwmPinStyleMotorFwdL, PinName pwmPinStyleMotorBackL, 
//         PinName pwmPinStyleMotorFwdR, PinName pwmPinStyleMotorBackR);

//         /**
//          * @brief Computes the pwm change (%) required to stay on black tape 
//          * 
//          * @return double - pwm change to stabilize direction, applied to the right if > 0, left if < 0
//          */
//         double pidBlackTape();
        
//         void drive();
//         void testDriveNoPID();
//         void testBackDriveNoPID();
//         void stopPwm();
//         void updateSensorVals();

//         // getters
//         int getErr();
//         int getDutyCycleL();
//         int getDutyCycleR();
//         bool getHasPwmChanged();

//         bool getOnTapeL();
//         bool getOnTapeM();
//         bool getOnTapeR();

//         // setters
//         void setDutyCycles(int newDutyCycleL, int newDutyCycleR);
//         void setHasPwmChanged(bool newStatus);
 
//     private:
//         void configPins();

//         // reflectance sensor pins (in)
//         int sensorL, sensorM, sensorR;
//         // pwm output pins (out)
//         int pwmMotorFwdL, pwmMotorBackL;
//         int pwmMotorFwdR, pwmMotorBackR;

//         PinName pwmPinStyleMotorFwdL, pwmPinStyleMotorBackL;
//         PinName pwmPinStyleMotorFwdR, pwmPinStyleMotorBackR;

//         bool direction; // (+) fwd, (-) back

//         // duty cycles for each pwm 
//         double dutyCycleL, dutyCycleR; // percentage
        
//         // PID control for black tape
//         volatile bool onTapeL, onTapeM, onTapeR;
        
//         bool prevOnTapeL, prevOnTapeR;
//         int err;
//         double p, d, i;
        
//         double prevErr;
//         double pwmChange;
//         bool hasPwmChanged;

//         // double setPt, dT, lastT;
// };
// #endif