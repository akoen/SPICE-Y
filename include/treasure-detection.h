#ifndef TREASURE_DETECTION
#define TREASURE_DETECTION

#include "tape-follower.h"
#include "sonar-sensor.h"

namespace TreasureDetection {
    // initial side sonar detection
    extern const double side_sonar_treasure_dists[6]; // cm
    extern const double side_sonar_treasure_dists_err[6]; // cm

    // front sonar detection
    extern const double front_sonar_treasure_dists[6]; // cm
    extern const double front_sonar_treasure_dists_err[6]; // cm

    // in claw detection
    extern const double treasure_in_claw_dist; // cm
    extern const double treasure_in_claw_dist_err; // cm

    extern const double def_drive_to_treasure_duty; // %
    
    // consecutive good readings needed
    extern const double side_sonar_req_good_readings[6];
    extern const double front_sonar_req_good_readings[6];
    extern const double claw_req_good_readings[6];

    // when near treasure
    extern const double near_treasure_dists[6];
    /**
     * Routine for collecting a treasure when tape following (first or second). 
     */ 
    bool obtainTapeTreasure(int treasureNum);

    /**
     * Collects the treasure upon initial detection. Returns to the original position if specified, and this is 
     * only possible if the cache is empty at the time of the method call (since it will execute all in cache)
     */
    bool treasureCollectionRoutine(Sonars::SonarType treasureLoc, double distFront, double distFrontErr, bool retOriginalPos, int treasureNum);
}
#endif
