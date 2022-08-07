#ifndef SPICEBOT_IR_H
#define SPICEBOT_IR_H

#include "motor-driver.h"
namespace IR {
    void getMagnitude(float magnitude[]);
    
    void driveWithPID();
    /**
     * Analogous to find black tape - finds the IR beacon.
     * Requires two good readings to be considered found, and the last reading will be used as the previoous p value when
     * using PID after.
     * 
     * A reading is considered good when the magnitudes of the two IR sensors are above the specified value,
     * and their differerence is within a specified value.
     * 
     * Returns true if IR is found.
     */ 
    bool findIR(double angle, int dutyCycle, Motors::RotateMode rotateMode, bool rotateRightFirst, double minMag, double maxDiff, int timeout);
}

#endif //SPICEBOT_IR_H
