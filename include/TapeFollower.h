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
    extern bool onTapeL;
    extern bool onTapeM;
    extern bool onTapeR;

    extern bool prevOnTapeL;
    extern bool prevOnTapeM;
    extern bool prevOnTapeR;

    extern int err;
    extern int prevErr;
 
    extern double p;
    extern double d;
    extern double i;

    extern double pwmChange;
    
    extern long currTime;
    extern long prevErrTime;

    extern bool startFlag;
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