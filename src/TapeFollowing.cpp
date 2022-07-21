// #include "PIDController.h"
#include "ReflectanceSensor.cpp"
#include "Encoder.cpp"

namespace TapeFollow {
    # define REFLECT_LOWER_THRES_BITS 300

    // PID tuning
    const double kp = 10;
    const double ki = 0;
    const double kd = 0;
    const double maxI = 0;

    // vars
    volatile bool onTapeL = false;
    volatile bool onTapeM = false;
    volatile bool onTapeR = false;

    volatile bool prevOnTapeL = false;
    volatile bool prevOnTapeM = false;
    volatile bool prevOnTapeR = false;

    volatile int err = 0;
    volatile int prevErr = 0;

    volatile double pwmChange = 0;  // due to PID

    volatile double p = 0;
    volatile double d = 0;
    volatile double i = 0;

    /**
     * Calculates PID for black tape
     * Returns: pwm change needed to adjust from current state, 
     * (+) for right wheel, (-) for left wheel
     */
    double calcPidBlackTape() {
        // read reflectance of all 3 sensors (0 or 1 for each)
        ReflectanceSensors::readFrontReflectanceSensors();

        onTapeL = !ReflectanceSensors::frontSensorLval;
        onTapeM = !ReflectanceSensors::frontSensorMval;
        onTapeR = !ReflectanceSensors::frontSensorRval;

        /* truth table: (-) = left, (+) = right
        * M true --> error=0, M R false, L true --> error=1, M R L false, prevL true --> error=2, 
        * M L false, R true --> error=-1, M L R false, prevR true --> error=-2
        */
        if (onTapeM) err = 0;
        else if (!onTapeL && onTapeR) err = -1;
        else if (!onTapeL && !onTapeR && prevOnTapeL) err = -2;
        else if (onTapeL && !onTapeR) err = 1;
        else if (!onTapeL && !onTapeR && prevOnTapeR) err = 2;
        else {}// should have no other states

        p = kp*err, d = kd*(err - prevErr), i += err;

        // anti windup
        if (i > maxI) {
            i = maxI;
        } else if (i < -maxI) {
            i = maxI;
        }
        pwmChange = p + d + i; // > 0 change for right, < 0 for left

        return pwmChange;
    }

    /**
     * Drives the robot with PID for the black tape
     */
    void driveWithPid() {
        // get error
        double changePwmSigned = calcPidBlackTape();
        // update pwm
        Motors::setDutyCycles(Motors::dutyCycleL - pwmChange, Motors::dutyCycleR + pwmChange);
        // drive (fwd)
        Motors::setDir(true, true);
        Motors::drive();        
    }

}