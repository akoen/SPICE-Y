#include "executor.h"
#include "treasure-detection.h"
#include "tape-follower.h"
#include "encoder.h"
namespace Executor {
    void execute() {
        // follow tape & obtain first treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(1);
        // back up - easier to find tape (and not worry for 1 1 1 instead of tape)
        Encoders::driveMotorsDistance(Motors::min_drive_dutyCycle, false, 20);
        // find tape 
        TapeFollow::findBlackTape(TapeFollow::DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS);
        // follow tape & obtain second treasure and come back to tape
        TreasureDetection::obtainTapeTreasure(2);
        
        // find tape
        // TapeFollow::findBlackTape(60);

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
}