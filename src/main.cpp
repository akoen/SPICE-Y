#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "PIDController.h"
// OLED standard setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// pins
# define PWM_MOTOR_L PB0
# define PWM_MOTOR_R PB1
# define PWM_FORMAT_MOTOR_L PB_0
# define PWM_FORMAT_MOTOR_R PB_1

# define SENSOR_PIN_L PB2
# define SENSOR_PIN_M PB3
# define SENSOR_PIN_R PB4

// const
const double maxVolt = 3.3;
// vars
volatile uint32_t loop_counter = 0;

PIDController pidController(SENSOR_PIN_L, SENSOR_PIN_M, SENSOR_PIN_R, 
PWM_MOTOR_L, PWM_MOTOR_R, PWM_FORMAT_MOTOR_L, PWM_FORMAT_MOTOR_R);

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

/**
 * @brief Initialize the analog in and out pins and the OLED display.
 * @param none
 * @retval none
 */
void setup() {
    //Initialize analog in pin
    OLEDSetup();
}

/**
 * @brief Read the value of the analog-in pin and set the timing for the LED
 *    	blink period depending on this value.
 * @param none
 * @retval none
 */
void loop() {
    display_handler.clearDisplay();
    display_handler.setCursor(0, 0);
    display_handler.display();

    pidController.drive();
    display_handler.println("DutyCycles(L,R): ");

    display_handler.print(pidController.getDutyCycleL());
    display_handler.print(", ");
    display_handler.println(pidController.getDutyCycleR());
}
