#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "MotorDriver.h"
#include "TapeFollower.h"
#include "Encoder.h"

// OLED standard setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile double refTime = 0;
volatile double currTime = 0;

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

using namespace Motors;
/**
 * @brief Initialize the OLED display.
 * @param none
 * @retval none
 */

void setup() {
    OLEDSetup();
    configMotorPins();
    Encoders::configEncoderPins();
}

void tapeFollowingPidTest();

/**
 * @brief Read the value of the analog-in pin and set the timing for the LED
 *    	blink period depending on this value.
 * @param none
 * @retval none
 */
void loop() {
    tapeFollowingPidTest();
}

void tapeFollowingPidTest() {
    display_handler.clearDisplay();
    display_handler.setCursor(0, 0);

    // TapeFollow::driveWithPid();
    Motors::drive();
    display_handler.print("Sensors(L,M,R): ");
    display_handler.print(TapeFollow::onTapeL);
    display_handler.print(" ");
    display_handler.print(TapeFollow::onTapeM);
    display_handler.print(" ");
    display_handler.println(TapeFollow::onTapeR);

    display_handler.print("PWM change: ");
    display_handler.println(TapeFollow::pwmChange);

    display_handler.print("Duty(L,R): ");
    display_handler.print(Motors::dutyCycleL);
    display_handler.print(" ");
    display_handler.println(Motors::dutyCycleR);
    display_handler.display();
}
