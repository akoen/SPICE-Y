#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"

void execute() {
    // follow tape & obtain first treasure and come back to tape
    TreasureDetection::obtainTapeTreasure(1);

    // find tape 
    TapeFollow::findBlackTape(60);

    // follow tape & obtain second treasure 
    TreasureDetection::obtainTapeTreasure(2);

    // find tape
    TapeFollow::findBlackTape(60);

    // fit through archway

    // obtain third treasure using IR PID

    // obtain fourth treasure using IR PID

    // IR PID until robot hits beacon

    // turn right 90 degs and drive until edge detected

    // back up some cm and deploy bridge

    // drive backwards to bridge using edge detection PID or black tape PID

    // obtain treasure 

    // get into location and drop box

    // orient robot and drive up bridge

    // obtain gold treasure

}