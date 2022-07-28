#include "tape-follower.h"
#include "encoder.h"

// PID tuning
const double TapeFollow::kp = 22;
const double TapeFollow::ki = 0;
const double TapeFollow::kd = 14;
const double TapeFollow::maxI = 100;
// chicken wire
const double TapeFollow::CHICKEN_WIRE_DIST = 17;
const double TapeFollow::DEF_TAPE_SEARCH_ANGLE = 30;
// vars
bool TapeFollow::onTapeL = false;
bool TapeFollow::onTapeM = false;
bool TapeFollow::onTapeR = false;
bool TapeFollow::prevOnTapeL = false;
bool TapeFollow::prevOnTapeM = false;
bool TapeFollow::prevOnTapeR = false;

int TapeFollow::err = 0;
int TapeFollow::prevErr = 0;

double TapeFollow::p = 0;
double TapeFollow::d = 0;
double TapeFollow::i = 0;
double TapeFollow::pwmChange = 0;
bool TapeFollow::startFlag = false;
long TapeFollow::prevErrTime = 0;
long TapeFollow::currTime = 0;

double TapeFollow::calcPidBlackTape() {
    // if (!startFlag) {
    //     prevErrTime = millis();
    //     currTime = prevErrTime;
    // } 
    // read reflectance of all 3 sensors (0 or 1 for each)
    ReflectanceSensors::readFrontReflectanceSensors();

    onTapeL = ReflectanceSensors::frontSensorLval;
    onTapeM = ReflectanceSensors::frontSensorMval;
    onTapeR = ReflectanceSensors::frontSensorRval;

    // chicken wire routine
    if (onTapeL && onTapeM && onTapeR) {   
        // pwmChange = CHICKEN_WIRE_OFFSET_DUTY;
        // Encoders::driveMotorsDistance(true, CHICKEN_WIRE_DIST);
        // return pwmChange;
        chickenWireRoutine();   
        // this updates preOnTape and onTape readings - can continue PID
    }

    /* discrete errors truth table: (-) = left, (+) = right */
    if (onTapeL && onTapeM) err = -1;
    else if (onTapeM && onTapeR) err = 1;
    else if (onTapeM) err = 0;
    else if (!onTapeL && onTapeR) err = -2;
    else if (!onTapeL && !onTapeR && prevOnTapeL) err = 5;
    else if (onTapeL && !onTapeR) err = 2;
    else if (!onTapeL && !onTapeR && prevOnTapeR) err = -5;
    else {
        // L M R all off tape, L R previously off tape -- do nothing
        // should have no other states
    }

    p = kp*err, d = kd*(err - prevErr), i += ki*err;

    // currTime = millis();
    // if (startFlag && err != 0) {
    //     // d = kd*(err - prevErr) / (1.0*(currTime - prevErrTime));   
    //     prevErrTime = currTime;     
    // } else {
    //     d = 0;
    //     startFlag = true;
    // }

    // anti windup
    if (i > maxI) {
        i = maxI;
    } else if (i < -maxI) {
        i = maxI;
    }
    pwmChange = p + d + i; // > 0 change for right, < 0 for left    // TODO: bad code - make local

    prevOnTapeL = onTapeL;
    prevOnTapeR = onTapeR;
    prevErr = err;

    return pwmChange;
}

void TapeFollow::driveWithPid() {
    // get error
    double changePwmSigned = calcPidBlackTape();

    // update pwm    
    Motors::setDutyCycles(Motors::dutyCycleL - changePwmSigned, Motors::dutyCycleR + changePwmSigned);
    // drive
    Motors::setDir(true, true);
    Motors::drive();        
}

void TapeFollow::chickenWireRoutine() {
    // drive across bridge
    Encoders::driveMotorsDistance(true, CHICKEN_WIRE_DIST);
    // find black tape
    findBlackTape(DEF_TAPE_SEARCH_ANGLE);
}

bool TapeFollow::findBlackTape(double angle) {
    // deg to pulses
    double anglePerPulse = 180 * (PI*Motors::WHEEL_DIAMETER / Encoders::pulse_per_rev) / Motors::WHEELS_WIDTH;
    int turnPulsesInterval = round(angle / anglePerPulse);
    
    int rotateCount = 0;

    int tapeReadingsCount;
    bool firstTapeReadingL;
    bool firstTapeReadingM;
    bool firstTapeReadingR;
    long startEncoderPulses;

    // rotate left - left wheel rotates back, right wheel at rest
    // rotate right after
    while (rotateCount < 2) {
        tapeReadingsCount = 0;
        firstTapeReadingL = false;
        firstTapeReadingM = false;
        firstTapeReadingR = false;

        if (rotateCount == 0) {
            startEncoderPulses = Encoders::pulseLW;
            Motors::rotateLeft();
        } else {
            startEncoderPulses = Encoders::pulseRW;
            Motors::rotateRight();
        }
        while (Encoders::pulseLW >= startEncoderPulses - turnPulsesInterval) {
            // look for tape while turning
            ReflectanceSensors::readFrontReflectanceSensors();
            if (!ReflectanceSensors::frontSensorLval && !ReflectanceSensors::frontSensorMval
            && !ReflectanceSensors::frontSensorRval) {
                if (tapeReadingsCount < 1) {
                    firstTapeReadingL = ReflectanceSensors::frontSensorLval;
                    firstTapeReadingM = ReflectanceSensors::frontSensorMval;
                    firstTapeReadingR = ReflectanceSensors::frontSensorRval;
                    
                    tapeReadingsCount++;
                } else {
                    // update reflectance values
                    prevOnTapeL = firstTapeReadingL;
                    prevOnTapeM = firstTapeReadingM;
                    prevOnTapeR = firstTapeReadingR;
                    onTapeL = ReflectanceSensors::frontSensorLval;
                    onTapeM = ReflectanceSensors::frontSensorMval;
                    onTapeR = ReflectanceSensors::frontSensorRval;
                    Motors::stopMotors();
                    return true;
                }
            }
        }
        Motors::stopMotors();
    }
    
    // didn't find tape
    Motors::stopMotors();    
    return false;
}
