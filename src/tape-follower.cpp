#include "tape-follower.h"
#include "encoder.h"

// PID tuning
double TapeFollow::kp = 23;
double TapeFollow::ki = 0;
double TapeFollow::kd = 15;
double TapeFollow::maxI = 100;
// chicken wire
const double TapeFollow::CHICKEN_WIRE_DIST = 17+7;
const double TapeFollow::DEF_TAPE_SEARCH_ANGLE = 45;
// vars
bool TapeFollow::crossedChickenWire = false;
bool TapeFollow::onTapeL = false;
bool TapeFollow::onTapeM = false;
bool TapeFollow::onTapeR = false;
bool TapeFollow::prevOnTapeL = false;
bool TapeFollow::prevOnTapeM = false;
bool TapeFollow::prevOnTapeR = false;
bool TapeFollow::checkChickenWire = true;
int TapeFollow::err = 0;
int TapeFollow::prevErr = 0;

double TapeFollow::p = 0;
double TapeFollow::d = 0;
double TapeFollow::i = 0;

double TapeFollow::pwmChange = 0;

double TapeFollow::calcPidBlackTape() {
    // read reflectance of all 3 sensors (0 or 1 for each)
    ReflectanceSensors::readFrontReflectanceSensors();
    ReflectanceSensors::printFrontReflectance();

    onTapeL = ReflectanceSensors::frontSensorLval;
    onTapeM = ReflectanceSensors::frontSensorMval;
    onTapeR = ReflectanceSensors::frontSensorRval;

    // chicken wire routine
    if ((onTapeL && onTapeM && onTapeR) && !crossedChickenWire) { 
        if (checkChickenWire) {
            Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, Motors::dutyCycleL, 50, Motors::default_motors_stop_millis, Motors::dutyCycleR - Motors::dutyCycleL);  
            chickenWireRoutine();
            crossedChickenWire = true;
        }

        // this updates prevOnTape and onTape readings - can continue PID
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
    // ReflectanceSensors::printFrontReflectance();
    // update pwm    
    Motors::setDutyCycles(Motors::dutyCycleL - changePwmSigned, Motors::dutyCycleR + changePwmSigned);
    // drive
    Motors::setDir(true, true);
    Motors::drive();        
}

void TapeFollow::driveWithPidDist(double dist) {
    int pulseInterval = Encoders::cmToPulses(dist);
    long startPulsesLW = Encoders::pulseLW;
    long startPulsesRW = Encoders::pulseRW;

    while (Encoders::pulseLW < startPulsesLW + pulseInterval && Encoders::pulseRW < startPulsesRW + pulseInterval) {
        driveWithPid();
    }
}
void TapeFollow::chickenWireRoutine() {
    // drive across bridge
    int dutyCycle = LW_PWM_DUTY;
    Encoders::driveMotorsDistance(dutyCycle, true, CHICKEN_WIRE_DIST);
    Motors::stopWithBrake(Motors::DRIVE_FWD, Motors::NONE, dutyCycle, 50);
    // Motors::stopWithBrake(Motors::DRIVE_FWD, Motors::NONE, Motors::dutyCycleL, 50, Motors::default_motors_stop_millis, Motors::dutyCycleR-Motors::dutyCycleL);
// 
    // find black tape - look left first
    findBlackTape(DEF_TAPE_SEARCH_ANGLE, Motors::min_rotate_dutyCycle, Motors::RotateMode::FORWARDS, false);
}

bool TapeFollow::findBlackTape(double angle, int dutyCycle, Motors::RotateMode rotateMode, bool rotateRightFirst, int timeout) {
    // deg to pulses
    double arcLen = rotateMode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
    double anglePerPulse = (PI * Motors::WHEEL_DIAMETER / Encoders::pulse_per_rev) / (PI / 180.0 * arcLen);
    int turnPulsesInterval = round(angle / anglePerPulse);
    int rotateCount = 0;

    int tapeReadingsCount;
    bool firstTapeReadingL;
    bool firstTapeReadingM;
    bool firstTapeReadingR;
    long startEncoderPulses;
    long checkEncoderPulses;

    long startMillis;
    long currMillis;

    // rotate left - left wheel rotates forwards, right wheel at rest
    // rotate right after - right wheel rotates forwards
    for (int rotateCount = 0; rotateCount < 2; rotateCount++) {
        tapeReadingsCount = 0;
        firstTapeReadingL = false;
        firstTapeReadingM = false;
        firstTapeReadingR = false;
        if (rotateCount == 0) {
            if (rotateRightFirst) {
                // search right
                if (rotateMode == Motors::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseRW;
                    checkEncoderPulses = Encoders::pulseRW;
                } else {
                    startEncoderPulses = Encoders::pulseLW;
                    checkEncoderPulses = Encoders::pulseLW;
                }
                Motors::rotate(Motors::default_rotate_pwm, true, rotateMode);
            } else {
                // search left
                if (rotateMode == Motors::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseLW;
                    checkEncoderPulses = Encoders::pulseLW;
                } else {
                    startEncoderPulses = Encoders::pulseRW;
                    checkEncoderPulses = Encoders::pulseRW;
                }
                Motors::rotate(Motors::default_rotate_pwm, false, rotateMode);
            }
            startMillis = millis();
            currMillis = startMillis;
        } else {   
            if (rotateRightFirst) {
                // search left
                if (rotateMode == Motors::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseLW;
                    checkEncoderPulses = Encoders::pulseLW;
                } else {
                    startEncoderPulses = Encoders::pulseRW;
                    checkEncoderPulses = Encoders::pulseRW;
                }
                Motors::rotate(Motors::default_rotate_pwm, false, rotateMode);
            } else {
                // search right
                if (rotateMode == Motors::BACKWARDS) {
                    startEncoderPulses = Encoders::pulseRW;
                    checkEncoderPulses = Encoders::pulseRW;
                } else {
                    startEncoderPulses = Encoders::pulseLW;
                    checkEncoderPulses = Encoders::pulseLW;
                }
                Motors::rotate(Motors::default_rotate_pwm, true, rotateMode);
                turnPulsesInterval = round(angle * 2 / anglePerPulse);    // twice the angle since needs to go left -> middle -> right
            }
            startMillis = millis();
            currMillis = startMillis;
        }
        while (true) {
            currMillis = millis();
            // loop end conditions
            if (rotateMode == Motors::BACKWARDS) {
                if (checkEncoderPulses < startEncoderPulses - turnPulsesInterval) break;
            } else {
                if (checkEncoderPulses > startEncoderPulses + turnPulsesInterval) break;
            }

            if (timeout != -1 && currMillis > startMillis + timeout*1000) break;

            // look for tape while turning - tape found when not 1 1 1 and at least one 1 is seen 
            ReflectanceSensors::readFrontReflectanceSensors();
            if (!(ReflectanceSensors::frontSensorLval && ReflectanceSensors::frontSensorMval
            && ReflectanceSensors::frontSensorRval) && (ReflectanceSensors::frontSensorLval || ReflectanceSensors::frontSensorMval
            || ReflectanceSensors::frontSensorRval)) {
                if (tapeReadingsCount == 0) {
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
                    Serial.println("found two");
                    // stop
                    if (rotateCount == 0) {
                        if (rotateRightFirst) {
                            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, rotateMode, dutyCycle, 50);
                        } else {
                            Motors::stopWithBrake(Motors::MotorAction::ROTATE_LEFT, rotateMode, dutyCycle, 50);
                        }
                    } else {
                        if (rotateRightFirst) {
                            Motors::stopWithBrake(Motors::MotorAction::ROTATE_LEFT, rotateMode, dutyCycle, 50);
                        } else {
                            Motors::stopWithBrake(Motors::MotorAction::ROTATE_RIGHT, rotateMode, dutyCycle, 50);
                        }
                    }
                    tapeReadingsCount++;
                    break;
                }
            }
            // update 
            if (rotateCount == 0) {
                if (rotateRightFirst) {
                    // search right
                    if (rotateMode == Motors::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseRW;
                    } else {
                        checkEncoderPulses = Encoders::pulseLW;
                    }
                } else {
                    // search left
                    if (rotateMode == Motors::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseLW;
                    } else {
                        checkEncoderPulses = Encoders::pulseRW;
                    }
                }
            } else {   
                if (rotateRightFirst) {
                    // search left
                    if (rotateMode == Motors::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseLW;
                    } else {
                        checkEncoderPulses = Encoders::pulseRW;
                    }
                } else {
                    // search right
                    if (rotateMode == Motors::BACKWARDS) {
                        checkEncoderPulses = Encoders::pulseRW;
                    } else {
                        checkEncoderPulses = Encoders::pulseLW;
                    }
                }
            }
        }
        // stop
        Motors::MotorAction turnAction;
        if (rotateCount == 0) {
            turnAction = rotateRightFirst ? Motors::MotorAction::ROTATE_RIGHT : Motors::MotorAction::ROTATE_LEFT; 
            Motors::stopWithBrake(turnAction, rotateMode, dutyCycle, 50);
        } else {
            turnAction = rotateRightFirst ? Motors::MotorAction::ROTATE_LEFT : Motors::MotorAction::ROTATE_RIGHT;
        }
        Motors::stopWithBrake(turnAction, rotateMode, dutyCycle, 50);

        // if found
        if (tapeReadingsCount == 2) return true;
    }

    return false;
}

void TapeFollow::chickenWireRoutine2(int prevErrEntering, int errEntering) {
    Motors::stopMotorsPWM();
    int crossingDuty = LW_PWM_DUTY;

    // first half - drive straight
    Encoders::driveMotorsDistance(LW_PWM_DUTY, true, CHICKEN_WIRE_DIST / 2.0);

    // second half -- drive straight in the other way, so rotate using the pwm change due to the inverse errors
    Motors::stopMotorsPWM(10);
    Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, crossingDuty, 50);
    // the pwm change due to PID when entering tape (using the input errs), + if right increase
    int pwmChange = kp*errEntering + kd*(errEntering-prevErrEntering);  

    // rotate to the opposite direction of incoming direction (the pwm change entering)
    Motors::rotate(crossingDuty, pwmChange > 0, Motors::RotateMode::FORWARDS);  // rotating left coming in -> so rotate left (pwm > 0), rotating right coming in -> so rotate right
    Motors::stopMotorsPWM(10);
    Motors::stopWithBrake(Motors::MotorAction::DRIVE_FWD, Motors::RotateMode::NONE, crossingDuty, 50);

    // drive straight
    Encoders::driveMotorsDistance(crossingDuty, true, CHICKEN_WIRE_DIST / 2.0);

    // now exiting chicken wire with the same error as incident, so set these errors  -- not sure about this (or find tape)
    // err = errEntering, prevErr = prevErrEntering;
    findBlackTape(30, Motors::min_rotate_dutyCycle, Motors::RotateMode::BOTH_WHEELS, false);
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