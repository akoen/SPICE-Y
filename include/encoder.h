#ifndef ENCODER
#define ENCODER

#include "config.h"
#include "motor-driver.h"
#include <Arduino.h>
#include <stack>
#include <tuple>
namespace Encoders {
    /* with div by 10 counter, there are */
    extern const double pulse_per_rev;  // divide by counter at end, increases pulse width

    extern volatile int interruptCountLW;
    extern volatile int interruptCountRW;

    extern volatile long pulseLW;
    extern double posLW;
    extern volatile long pulseRW;
    extern double posRW;

    extern volatile long cacheStartPulse;
    extern volatile long cacheEndPulse;

    // + for fwd, - for back
    // drive action, rotate mode, duty cycle, pulse interval
    extern std::stack<std::tuple<Motors::MotorAction, Motors::RotateMode, int, int>*>* cachedActions;  

    extern bool cacheCreated;
    extern bool cacheAddInProgress;
    /**
     * Interrupt service routine (ISR) that will be called each (left) encoder pulse
     */
    void ISR_LW();
    /**
     * Interrupt service routine (ISR) that will be called each (right) encoder pulse
     */
    void ISR_RW();

    void attachInterrupts();
    void configEncoderPins();
    void detachEncoderInterrupts();
    void resetEncoderVals();

    /**
     * Caches an action of encoder pulses. An action may be the following: drive fwd (both +), drive back (both -), 
     * rotate left (left +, right -), rotate right (left -, right +). Each cached spot should only represent one action.
     * 
     * The entire cache the specific combination of actions the robot has taken from the moment created.
     * Recommended to use when the motors have stopped moving so that encoder value don't change while function call - which 
     * sets reference encoder values for the cache.
     * 
     * Action is added to the cache once it has ended and must be ended prior to starting a new action.
     * Returns true if above statement is satisfed. 
     */ 
    bool startAddActionCache(Motors::MotorAction motorAction, Motors::RotateMode rotateMode, int dutyCycle);
    
    /**
     * Current action in progress is ended and added to the cache.
     * Returns true if an action was in progress at the time of the call.
     */
    bool endAddActionCache();
    /**
     * Executes the combination of encoder pulses in reverse of the stored cache, until empty.
     * The cache is destroyed afterwards.
     * The delay between each action (in ms) is also to be specified.
     * 
     * Returns true if the action can be executed
     */ 
    bool executeReverseCache(int actionDelayMillis=100);

    /**
     * Drives the motors for the given interval of pulses.
     * If pulse interval for a wheel < 0, that wheel drives backwards and > 0 for forwrads.
     * Motors are stopped after execution, with no delay
     * 
     * This method is blocking (i.e. all other processes must wait until this finished)
     */ 
    void driveMotorsEncoderPulses(int dutyCycle, Motors::MotorAction motorAction, Motors::RotateMode rotateMode, int pulseInterval);

    /* degs to pulses: assume arc length = distance of wheel driven (no slipping)
     * dist = arc length = (deg / 180.0) * wheel width
     * deg = 180 * dist / wheel width
     * 
     * dist to pulses: 
     * distance per pulse = pi*diameter / pulse per rev
     * pulses = dist / distances per pulse
     * 
     * deg per pulse = 180 * (pi*diameter / pulse per rev) / wheel width
     * pulses = deg / deg per pulse 
     */

    /**
     * Drives the motors fwd (true) or back (false) a certain distance (cm)
     * 
     * This method is blocking (i.e. all other processes must wait until this finished)
     */ 
    void driveMotorsDistance(int dutyCycle, bool dirFwd, double distance);

    /**
     * Rotates the motors right (true) or left (false) a certain angle
     */ 
    void rotateMotorsDegs(int dutyCycle, bool dirRight, Motors::RotateMode mode, double angle);

    /**
     * Stops the motors when desired using encoders. This method puts the motors at rest within a threshold of specified set point pulses.   
     */
    void stopMotorsBrakeEncoders(Motors::MotorAction initialAction, Motors::RotateMode initialRotateMode, long setPtPulseLW, long setPtPulsesRW, int initialDutyCycle, int pulsesThreshold);

}
#endif