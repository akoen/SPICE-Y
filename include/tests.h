#include "config.h"
/*
 * This file is meant to implement a test framework for the robot, containing tests for both
 * existing and new implementation.
 * 
 * Regression testing should be done after a new feature has been implmenented, to ensure that the feature is
 * compatible with the robot's existing functionality.
 */

namespace Tests {
    void straightLine(int dutyCycle);

    void rotate90degs();
    void driveFwd20cm();
    void driveBack20cm();
    
    void servoLocationsTest();

    void sonarSensorTest();

    void encoderCacheTest();
}