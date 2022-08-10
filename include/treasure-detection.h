#ifndef TREASURE_DETECTION
#define TREASURE_DETECTION

#include "tape-follower.h"
#include "sonar-sensor.h"

namespace TreasureDetection {
    /* For sonar */
    // initial side sonar detection
    extern const double side_sonar_treasure_dists[6]; // cm
    extern const double side_sonar_treasure_dists_err[6]; // cm

    // front sonar detection
    extern const double front_sonar_treasure_dists[6]; // cm
    extern const double front_sonar_treasure_dists_err[6]; // cm

    // in claw detection
    extern const double treasure_in_claw_dist; // cm
    extern const double treasure_in_claw_dist_err; // cm

    // in v detection
    extern const double treasure_in_v_dist;

    // consecutive good readings needed
    extern const int side_sonar_req_good_readings[6];
    extern const int front_sonar_req_good_readings[6];
    extern const int claw_req_good_readings[6];

    // driving duty cycle
    extern const double def_drive_to_treasure_duty; // %
    
    /* For encoder */
    extern const double v_to_claw_dist;
    extern const double near_treasure_dists[6];

    /**
     * Routine for collecting a treasure when tape following (1st or 2nd). 
     */ 
    bool obtainTapeTreasure(int treasureNum, bool retToOriginalPos);

    /**
     * Routine for collecting a treasure when following IR (3rd or 4th)
     */ 
    bool obtainIRTreasure(int treasureNum, bool cache);

    /**
     * Routine for collecting the third treasure during IR PID. Requires that the robot can reliably start IR 
     * PID at the time of the method call.
     */ 
    bool obtainThirdIRtreasure(double driveFwd, double rotateLeftDegs, int driveDuty, bool cache);
    
    /**
     * Routine for collecting the fourth treasure after the robot has fully crossed the bridge.
     */ 
    bool obtainFifthTreasure(int driveDuty, double rotateRightDegs, double driveBackDist, bool cache);
    /**
     * Collects the treasure upon initial detection. Returns to the original position if specified, and this is 
     * only possible if the cache is empty at the time of the method call (since it will execute all in cache)
     */
    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum);

    /**
     * Drives to the treasure when the front sonar has detected it, given the distance of the robot at the time of the method call.
     * Reecalibrates the treasure within V range prior to collection routine if needed.
     * 
     * true if in v, false if in rocks
     */
    bool driveToTreasureFrontSonar(double initialDist, int treasureNum, int timeout, bool retOriginalPos, int dutyCycle=def_drive_to_treasure_duty);

    bool driveToTreasureFrontSonarIR3(double initialDist, int reqGoodReadings, int timeout);
    
    void regularDriveToTreasureFront(int treasureNum, int timeout); 

    /**
     * A failsafe function for when the treasure has not been detected. Looks for the treasure by searching
     * within the specified angle.
     *  
     * Returns true if treasure found. The robot will be stopped in the location that the treasure was found post-calibration, 
     * so it is externally required to position the robot into a position for treasure pickup
     * If not found, returns to the original position but backed up for rotation.
     */
    // bool treasureCalibration(double backUpDist, double turnDegs, double expectedMaxDist, int reqGoodReadings);
    bool treasureCalibration(double backUpDist, double turnDegs, double expectedMaxDist, int treasureNum);
}
#endif