#ifndef TAPE_FOLLOWER
#define TAPE_FOLLOWER

#include "motor-driver.h"
#include "reflectance-sensor.h"

namespace TapeFollow {
    // PID tuning
    extern const double kp;
    extern const double ki;
    extern const double kd;
    extern const double maxI;
    // chicken wire (cm)
    extern const double CHICKEN_WIRE_DIST;
    // degs both rotations
    extern const double DEF_TAPE_SEARCH_ANGLE;

    // dist b/w wheels (cm)
    extern const double WHEELS_WIDTH;
    // vars
    extern bool crossedChickenWire;

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

    /**
     * Drives the robot with PID for the specified distance
     */
    void driveWithPidDist(double dist);
    /**
     * To be ran when encountered a chicken wire (sensors see all 1 1 1)
     * 
     */ 
    void chickenWireRoutine();
    
    /**
     * Attempts to find the black tape, with a specified search angle (deg) in front.
     * 
     * If tape is found, as in, took two "on tape" readings --> updates prevOnTape and onTape vals
     * 
     * Reliance:
     * 1) Never "1 1 1" when not on chicken wire 
     * 2) sensors collect at least two "on tape" (non 0 0 0) readings when turning. This may imply we need to turn slowly
     * Returns true if tape found
     */
    bool findBlackTape(double angle, int dutyCycle, Motors::RotateMode rotateMode);

    /**
     * To be called when 1 1 1 is read by sensors.
     * 
     * prevErr, err are the result of PID error obtained just prior to 1 1 1 encounter
     * The motors are driving as a result of the PID from this error
     * 
     * Edge of chicken wires show uncertain results
     * Chicken wire (1 1 1) to be set to drive straight (ref duty)
     * For the first half of the chicken wire, drive with pwm of the err from the reading before chicken wire
     * Second half of the chicken wire, drive with the opposite to that error
     * After chicken wire, continue pid with the second half error. May also consider calling find black tape method
     *
     * Note: doesn't do integral for inverse PWM change
     */ 
    void chickenWireRoutine2(int prevErr, int err);

    bool chickenWireCrossNonEncoder(int rotateTime);
}
#endif