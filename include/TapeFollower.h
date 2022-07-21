#ifndef TapeFollowingFile
#define TapeFollowingFile

#include "MotorDriver.h"
#include "ReflectanceSensor.h"

namespace TapeFollow {
    // PID tuning
    extern const double kp;
    extern const double ki;
    extern const double kd;
    extern const double maxI;

    // vars
    extern volatile bool onTapeL;
    extern volatile bool onTapeM;
    extern volatile bool onTapeR;

    extern volatile bool prevOnTapeL;
    extern volatile bool prevOnTapeM;
    extern volatile bool prevOnTapeR;

    extern volatile int err;
    extern volatile int prevErr;
 
    extern volatile double pwmChange;  // due to PID

    extern volatile double p;
    extern volatile double d;
    extern volatile double i;

    /**
     * Calculates PID for black tape
     * Returns: pwm change needed to adjust from current state, 
     * (+) for right wheel, (-) for left wheel
     */
    double calcPidBlackTape();

    /**
     * Drives the robot with PID for the black tape
     */
    void driveWithPid();
}

#endif