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
    if ((onTapeL && onTapeM && onTapeR)) {   
        chickenWireRoutine();
        // chickenWireRoutine2(prevErr, err);   

        // this updates prevOnTape and onTape readings - can continue PID

        // Motors::setDir(true, true);
        // Motors::setDutyCycles(20, 20 + Motors::default_motors_offset + CHICKEN_WIRE_OFFSET_DUTY);
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
    int dutyCycle = LW_PWM_DUTY - 10;
    Encoders::driveMotorsDistance(dutyCycle, true, CHICKEN_WIRE_DIST);
    // Encoders::stopMotorsBrakeEncoders(Motors::DRIVE_FWD, Motors::RotateMode::NONE, Encoders::pulseLW, Encoders::pulseRW, dutyCycle, 131);
    delay(10);
    Motors::stopWithBrake(Motors::DRIVE_FWD, Motors::NONE, LW_PWM_DUTY, 50);

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
    long checkEncoderPulses;

    Motors::RotateMode rotateMode = Motors::RotateMode::FORWARDS;
    // rotate right - left wheel rotates forwards, right wheel at rest
    // rotate left after
    while (rotateCount < 2) {
        tapeReadingsCount = 0;
        firstTapeReadingL = false;
        firstTapeReadingM = false;
        firstTapeReadingR = false;

        if (rotateCount == 0) {
            startEncoderPulses = Encoders::pulseRW;
            checkEncoderPulses = Encoders::pulseRW;
            // Motors::rotateLeft();
            Motors::rotate(Motors::default_rotate_pwm, false, rotateMode);
        } else {
            startEncoderPulses = Encoders::pulseLW;
            checkEncoderPulses = Encoders::pulseLW;
            // Motors::rotateRight();
            Motors::rotate(Motors::default_rotate_pwm, true, rotateMode);
            turnPulsesInterval *= 2;    // twice the angle since needs to go left -> middle -> right
        }
        while (checkEncoderPulses <= startEncoderPulses + turnPulsesInterval) {
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

                    // // stop
                    // if (rotateCount == 0) {
                    //     // Encoders::stopMotorsBrakeEncoders(Motors::ROTATE_LEFT, rotateMode, Encoders::pulseLW, Encoders::pulseRW, Motors::default_rotate_pwm, 131);
                    //     Motors::stopWithBrake(Motors::ROTATE_LEFT, rotateMode, Motors::default_rotate_pwm, 50);
                    // } else {
                    //     // Encoders::stopMotorsBrakeEncoders(Motors::ROTATE_RIGHT, rotateMode, Encoders::pulseLW, Encoders::pulseRW, Motors::default_rotate_pwm, 131);
                    //     Motors::stopWithBrake(Motors::ROTATE_RIGHT, rotateMode, Motors::default_rotate_pwm, 50);
                    // }
                    // return true;
                }
                break;
            }
            // update 
            checkEncoderPulses = rotateCount == 0 ? Encoders::pulseLW : Encoders::pulseRW;
        }
        // stop
        if (rotateCount == 0) {
            // Encoders::stopMotorsBrakeEncoders(Motors::ROTATE_LEFT, rotateMode, Encoders::pulseLW, Encoders::pulseRW, Motors::default_rotate_pwm, 131);
            Motors::stopWithBrake(Motors::ROTATE_LEFT, rotateMode, Motors::default_rotate_pwm, 50);
        } else {
            // Encoders::stopMotorsBrakeEncoders(Motors::ROTATE_RIGHT, rotateMode, Encoders::pulseLW, Encoders::pulseRW, Motors::default_rotate_pwm, 131);
            Motors::stopWithBrake(Motors::ROTATE_RIGHT, rotateMode, Motors::default_rotate_pwm, 50);
        }
    }
    return tapeReadingsCount == 2;
}

void TapeFollow::chickenWireRoutine2(int prevErrEntering, int errEntering) {
    Motors::stopMotorsPWM();
    // first half - drive straight
    Encoders::driveMotorsDistance(LW_PWM_DUTY - 10, true, CHICKEN_WIRE_DIST / 2.0);

    // second half -- drive straight in the other way, so rotate using the pwm change due to the inverse errors
    Motors::stopMotorsPWM();
    // the pwm change due to PID when entering tape (using the input errs), + if right increase
    int pwmChange = kp*errEntering + kd*(errEntering-prevErrEntering);  
    // rotate to the opposite direction of incoming direction (the pwm change entering)
    if (pwmChange > 0) Motors::rotateRight(pwmChange);  // was rotating left coming in, so rotate left
    else Motors::rotateRight(-pwmChange);   // was rotating right coming in, so rotate left
    Motors::stopMotorsPWM();

    // drive straight
    Encoders::driveMotorsDistance(LW_PWM_DUTY - 10, true, CHICKEN_WIRE_DIST / 2.0);

    // now exiting chicken wire with the same error as incident, so set these errors  -- not sure about this
    err = errEntering, prevErr = prevErrEntering;
}

// bool TapeFollow::chickenWireCrossNonEncoder(int rotateTime) {
//     // cross 
//     Motors::setDir(true, true);
//     Motors::setDutyCycles(LW_PWM_DUTY, RW_PWM_DUTY);
//     Motors::drive();

//     do {
//         ReflectanceSensors::readFrontReflectanceSensors();
//         onTapeL = ReflectanceSensors::frontSensorLval;
//         onTapeM = ReflectanceSensors::frontSensorLval;
//         onTapeR = ReflectanceSensors::frontSensorLval;
//     } while (onTapeL && onTapeM && onTapeR);
//     Motors::stopMotorsPWM(1000);

//     // do {
//     //     ReflectanceSensors::readFrontReflectanceSensors();
//     //     onTapeL = ReflectanceSensors::frontSensorLval;
//     //     onTapeM = ReflectanceSensors::frontSensorLval;
//     //     onTapeR = ReflectanceSensors::frontSensorLval;
//     // } while (onTapeL && onTapeM && onTapeR);
//     int rotateCount = 0;

//     int tapeReadingsCount;
//     bool firstTapeReadingL;
//     bool firstTapeReadingM;
//     bool firstTapeReadingR;
//     // rotate until tape found
//     // left
//     Motors::rotateLeft();
//     long prevTime = millis();
//     long currTime = prevTime;
//     while (currTime < prevTime + rotateTime) {
//         ReflectanceSensors::readFrontReflectanceSensors();
//         onTapeL = ReflectanceSensors::frontSensorLval;
//         onTapeM = ReflectanceSensors::frontSensorLval;
//         onTapeR = ReflectanceSensors::frontSensorLval;

//     }
//     int rotateCount = 0;
//     // rotate left - left wheel rotates back, right wheel at rest
//     // rotate right after
//     while (rotateCount < 2) {
//         tapeReadingsCount = 0;
//         firstTapeReadingL = false;
//         firstTapeReadingM = false;
//         firstTapeReadingR = false;

//         if (rotateCount == 0) {
//             Motors::rotateLeft();
//         } else {
//             Motors::rotateRight();
//         }
//         while (Encoders::pulseLW >= startEncoderPulses - turnPulsesInterval) {
//             // look for tape while turning
//             ReflectanceSensors::readFrontReflectanceSensors();
//             if (!ReflectanceSensors::frontSensorLval && !ReflectanceSensors::frontSensorMval
//             && !ReflectanceSensors::frontSensorRval) {
//                 if (tapeReadingsCount < 1) {
//                     firstTapeReadingL = ReflectanceSensors::frontSensorLval;
//                     firstTapeReadingM = ReflectanceSensors::frontSensorMval;
//                     firstTapeReadingR = ReflectanceSensors::frontSensorRval;
                    
//                     tapeReadingsCount++;
//                 } else {
//                     // update reflectance values
//                     prevOnTapeL = firstTapeReadingL;
//                     prevOnTapeM = firstTapeReadingM;
//                     prevOnTapeR = firstTapeReadingR;
//                     onTapeL = ReflectanceSensors::frontSensorLval;
//                     onTapeM = ReflectanceSensors::frontSensorMval;
//                     onTapeR = ReflectanceSensors::frontSensorRval;
//                     Motors::stopMotorsPWM();
//                     return true;
//                 }
//             }
//         }
//         Motors::stopMotorsPWM();
//     }
    
//     // didn't find tape
//     Motors::stopMotorsPWM();    
//     return false;
// }