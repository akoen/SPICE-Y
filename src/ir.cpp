#include "ir.h"
#include "board-setup.h"
#include "helpers.h"
#include "encoder.h"

namespace IR {
    const int targetFrequency = 10000;
    const float sampleFrequency = (12000000 / (12.5 + 71.5)) / 2;
    const float kp = 2;
    const float kd = 20;

    float magSmoothed[] = {0, 0};
    float prevP = 0;
    float p = 0;
    float d = 0;
    
    bool getMagnitudeInner(float magnitude[]) {
        if(!DMA1DataAvailable) return false;
        uint16_t dataL[IR_SENS_NUM_READINGS / 2] = {0};
        uint16_t dataR[IR_SENS_NUM_READINGS / 2] = {0};
        uint16_t *source[] = {dataL, dataR};

        deinterleave<uint16_t>(DMA1Data, source, IR_SENS_NUM_READINGS, 2);
        magnitude[0] = goertzelMag(IR_SENS_NUM_READINGS / 2, targetFrequency, sampleFrequency, dataL);
        magnitude[1] = goertzelMag(IR_SENS_NUM_READINGS / 2, targetFrequency, sampleFrequency, dataR);

        return true;
    }

    void getMagnitude(float magnitude[]) {
        bool success;
        do {
            success = getMagnitudeInner(magnitude);
        }  while(!success);

        DMA1DataAvailable = false;
        HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
        // Serial.print(magnitude[0]);
        // Serial.print(" ");
        // Serial.println(magnitude[1]);
        // while(!Serial);
        // Serial.write((uint8_t *) magnitude, 8);
    }

    float calcPID() {
        float magnitude[2];
        getMagnitude(magnitude);

        p = (magnitude[1] - magnitude[0]) / 100;
        d = p - prevP;

        prevP = p;

        return kp*p + kd*d;
    }

    void driveWithPID() {
        float pid = calcPID();
        // while(!Serial);
        // Serial.write((uint8_t *) &pid, 4);
        Motors::setDutyCycles(Motors::dutyCycleL + pid, Motors::dutyCycleR - pid);
        Motors::setDir(true, true);
        Motors::drive();
    }

    bool findIR(double angle, int dutyCycle, Motors::RotateMode rotateMode, bool rotateRightFirst, double minMag, double maxDiff, int timeout) {
        // readings
        float mag[2];
        // deg to pulses
        double arcLen = rotateMode == Motors::RotateMode::BOTH_WHEELS ? Motors::WHEELS_WIDTH / 2.0 : Motors::WHEELS_WIDTH;
        double anglePerPulse = (PI * Motors::WHEEL_DIAMETER / Encoders::pulse_per_rev) / (PI / 180.0 * arcLen);
        int turnPulsesInterval = round(angle / anglePerPulse);
        int rotateCount = 0;

        long startMillis;
        long currMillis;
        
        long startEncoderPulses;
        long checkEncoderPulses;
        int IRreadingsCount = 0;

        bool currTurnRight;
        // rotate left - left wheel rotates forwards, right wheel at rest
        // rotate right after - right wheel rotates forwards
        for (int rotateCount = 0; rotateCount < 2; rotateCount++) {
            IRreadingsCount = 0;
            if (rotateCount == 0) {
                if (rotateRightFirst) {
                    // search right
                    if (rotateMode == Motors::BACKWARDS) {
                        startEncoderPulses = Encoders::pulseRW;
                    } else {
                        startEncoderPulses = Encoders::pulseLW;
                    }
                    currTurnRight = true;
                } else {
                    // search left
                    if (rotateMode == Motors::BACKWARDS) {
                        startEncoderPulses = Encoders::pulseLW;
                    } else {
                        startEncoderPulses = Encoders::pulseRW;
                    }
                    currTurnRight = false;
                }
            } else {   
                if (rotateRightFirst) {
                    // search left
                    if (rotateMode == Motors::BACKWARDS) {
                        startEncoderPulses = Encoders::pulseLW;
                    } else {
                        startEncoderPulses = Encoders::pulseRW;
                    }
                    currTurnRight = false;
                } else {
                    // search right
                    if (rotateMode == Motors::BACKWARDS) {
                        startEncoderPulses = Encoders::pulseRW;
                    } else {
                        startEncoderPulses = Encoders::pulseLW;
                    }
                    currTurnRight = true;
                    turnPulsesInterval = round(angle * 2 / anglePerPulse);    // twice the angle since needs to go left -> middle -> right
                }
            }
            Motors::rotate(Motors::default_rotate_pwm, currTurnRight, rotateMode);
            checkEncoderPulses = startEncoderPulses;
            startMillis = millis();
            currMillis = startMillis;

            while (true) {
                currMillis = millis();
                // loop end conditions
                if (rotateMode == Motors::BACKWARDS) {
                    if (checkEncoderPulses < startEncoderPulses - turnPulsesInterval) break;
                } else {
                    if (checkEncoderPulses > startEncoderPulses + turnPulsesInterval) break;
                }

                if (timeout != -1 && currMillis > startMillis + timeout*1000) break;

                // look for IR while turning - IR found when both at sufficiently high magnitude & equal
                getMagnitude(mag);

                if (mag[0] > minMag && mag[1] > minMag && abs(mag[0]-mag[1]) < maxDiff) {
                    if (IRreadingsCount == 0) {
                        IRreadingsCount++;
                    } else {
                        // update IR values
                        p = (mag[1] - mag[0]) / 100;
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
                        IRreadingsCount++;
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
            if (IRreadingsCount == 2) return true;
        }
        return false;
    }
}
