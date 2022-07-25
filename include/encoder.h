#ifndef ENCODER
#define ENCODER

#include "config.h"
#include "motor-driver.h"
#include <Arduino.h>
#include <stack>

namespace Encoders {
    extern const double pulse_per_rev;  // divide by counter at end, increases pulse width
    extern const double wheel_diameter; // cm

    extern volatile int interruptCountLW;
    extern volatile int interruptCountRW;

    extern volatile long pulseLW;
    extern double posLW;
    extern volatile long pulseRW;
    extern double posRW;

    extern volatile long cacheStartPulseLW;
    extern volatile long cacheStartPulseRW;

    extern volatile long cacheEndPulseLW;
    extern volatile long cacheEndPulseRW;

    // + for fwd, - for back
    extern std::stack<int> *cachedActionsLeftPulses;  
    extern std::stack<int> *cachedActionsRightPulses;

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
    bool startAddActionCache();
    
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
     */ 
    void driveMotorsEncoderPulses(int pulseIntervalLW, int pulseIntervalRW);
}
#endif