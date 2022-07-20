#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "PIDController.h"
#include "SensorReader.h"
#include <Pins.h>

// OLED standard setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// const
const double maxVolt = 3.3;
// pulse per rev
const double pulsePerRev = 1389.9185 / 10.0;  // divide by counter at end, increases pulse width
const double wheelDiameter = 6.4; // cm
const double changeDirTime = 7; // s

// vars
volatile uint32_t interruptCountLW = 0;
volatile uint32_t interruptCountRW = 0;

volatile long pulseLW = 0;
volatile int dirLW = 0;
volatile double posLW = 0;

volatile long pulseRW = 0;
volatile int dirRW = 0;
volatile double posRW = 0;
volatile bool isDirFwd = true;

// PIDController pidController((int)SENSOR_PIN_L, (int)SENSOR_PIN_M, (int)SENSOR_PIN_R, 
// (int)PWM_MOTOR_FWD_L, (int)PWM_MOTOR_BACK_L, (int)PWM_MOTOR_FWD_R, (int)PWM_MOTOR_BACK_R, PWM_FORMAT_MOTOR_FWD_L, PWM_FORMAT_MOTOR_BACK_L,
// PWM_FORMAT_MOTOR_FWD_R, PWM_FORMAT_MOTOR_BACK_R);

volatile double refTime = 0;
volatile double currTime = 0;

volatile bool isOnTapeL = false;
volatile bool isOnTapeM = false;
volatile bool isOnTapeR = false;

// test
SensorReader sensorReader(SENSOR_PIN_L, SENSOR_PIN_M, SENSOR_PIN_R);

/**
 * @brief Sets up the OLED display
 */
void OLEDSetup() {
    //Initialize OLED display
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
    // Displays Adafruit logo by default. Call clearDisplay immediately if you don't want this.
    display_handler.display();
    delay(2000);

    // Displays "Hello world!" on the screen
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("Hello world!");
    display_handler.display();
}

void checkPosLW(int pin1) {
    int state1 = digitalRead(pin1);
    if (state1 == 0) {
        pulseLW = -99999; // shouldn't occur
        dirLW = -1;
    } else {    // CCW
        if (isDirFwd) {
            pulseLW++;
        } else {
            pulseLW--;
        }
        dirLW = isDirFwd;
    } 
    interruptCountLW++;
}

void checkPosRW(int pin1) {
    int state1 = digitalRead(pin1);
    if (state1 == 0) {
        dirRW = -99999; // shouldn't occur
        dirRW = -1;
    } else {    // CCW
        if (isDirFwd) {
            pulseRW++;
        } else {
            pulseRW--;
        }
        dirRW = isDirFwd;
    } 

    interruptCountRW++;
}

void ISR_LeftWheel() {
    checkPosLW(L_ENCODER_PIN1);
}

void ISR_RightWheel() {
    checkPosRW(R_ENCODER_PIN1);
}
bool changedFlag = false;
bool firstLoopFlag = true;

/**
 * @brief Initialize the OLED display.
 * @param none
 * @retval none
 */
void setup() {
    OLEDSetup();
    // attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1), ISR_LeftWheel, RISING); // ISR called when pin signal changes 0->1
    // attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1), ISR_RightWheel, RISING); // ISR called when pin signal changes 0->1
    // attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN2), checkPos, CHANGE);
}

bool onTapeL, onTapeM, onTapeR;
/**
 * @brief Read the value of the analog-in pin and set the timing for the LED
 *    	blink period depending on this value.
 * @param none
 * @retval none
 */
void loop() {
    
    // // motor control
    // if (firstLoopFlag) {
    //     refTime = millis();
    //     currTime = refTime;
    //     firstLoopFlag = false;
    //     return;
    // }
    // currTime = (millis() - refTime) / 1000.0;

    display_handler.clearDisplay();
    display_handler.setCursor(0, 0);

    // if (currTime < changeDirTime) {
    //     pidController.testDriveNoPID(); 

    //     isDirFwd = true;
    // } else if (!changedFlag) {
    //     pidController.stopPwm();
    //     delay(3000);
    //     pidController.testBackDriveNoPID();
    //     changedFlag = true;

    //     isDirFwd = false;
    // }

    // display_handler.print("DutyC(L,R): ");
    // display_handler.print(pidController.getDutyCycleL());
    // display_handler.print(", ");
    // display_handler.println(pidController.getDutyCycleR());

    // display_handler.print("Time passed (s): ");
    // display_handler.println(currTime);

    // // encoder after interrupt
    // posLW = (pulseLW / pulsePerRev) * PI*wheelDiameter;
    // posRW = (pulseRW / pulsePerRev) * PI*wheelDiameter;

    // display_handler.println("Pos cm(L,R):  ");
    // display_handler.print(posLW);
    // display_handler.print(", ");
    // display_handler.println(posRW);

    // display_handler.print("Dir(L,R):  ");
    // display_handler.print(dirLW);
    // display_handler.print(", ");
    // display_handler.println(dirRW);

    // read sensors
    // pidController.updateSensorVals();

    // onTapeL = digitalRead(SENSOR_PIN_L);
    // onTapeM = digitalRead(SENSOR_PIN_M);
    // onTapeR = digitalRead(SENSOR_PIN_R);
    
    // display_handler.println("Sensors(L,M,R): ");
    // display_handler.print(pidController.getOnTapeL());
    // display_handler.print(" ");
    // display_handler.print(pidController.getOnTapeM());
    // display_handler.print(" ");
    // display_handler.print(pidController.getOnTapeR());
    sensorReader.readSensors();
    display_handler.println("Sensors(L,M,R): ");
    display_handler.print(sensorReader.sensorLval);
    display_handler.print(" ");
    display_handler.print(sensorReader.sensorMval);
    display_handler.print(" ");
    display_handler.print(sensorReader.sensorRval);

    // display_handler.println("Sensors(L,M,R): ");
    // display_handler.print(onTapeL);
    // display_handler.print(" ");
    // display_handler.print(onTapeM);
    // display_handler.print(" ");
    // display_handler.print(onTapeR);

    // display_handler.println("ISR(L,R):  ");
    // display_handler.print(interruptCountLW);
    // display_handler.print(", ");
    // display_handler.println(interruptCountRW);    

    display_handler.display();

}
