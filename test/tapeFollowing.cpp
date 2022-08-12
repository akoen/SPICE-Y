// /**
//  * This file is responsible for all tape following purposes.
//  */

// #include <Wire.h>
// #include <Adafruit_SSD1306.h>

// // OLED standard setup
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET 	-1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// // pins
// #define TAPE_IN_PIN PB0

// // const
// const double maxVolt = 3.3;
// // vars
// volatile uint32_t loop_counter = 0;
// volatile double voltVal = 0;

// /**
//  * @brief Sets up the OLED display
//  */
// void OLEDSetup() {
//     //Initialize OLED display
//     display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
//     // Displays Adafruit logo by default. Call clearDisplay immediately if you don't want this.
//     display_handler.display();
//     delay(2000);

//     // Displays "Hello world!" on the screen
//     display_handler.clearDisplay();
//     display_handler.setTextSize(1);
//     display_handler.setTextColor(SSD1306_WHITE);
//     display_handler.setCursor(0,0);
//     display_handler.println("Hello world!");
//     display_handler.display();
// }

// /**
//  * @brief Initialize the analog in and out pins and the OLED display.
//  * @param none
//  * @retval none
//  */
// void setup() {
//     // set analog resolution to 12 bits, as Arduion lib decreases to 10
//     // analogReadResolution(12);
//     //Initialize analog in pin
//     pinMode(TAPE_IN_PIN, INPUT);
//     OLEDSetup();
// }

// /**
//  * @brief Read the value of the analog-in pin and set the timing for the LED
//  *    	blink period depending on this value.
//  * @param none
//  * @retval none
//  */
// void loop() {
//     voltVal = analogRead(TAPE_IN_PIN);
//     voltVal =  voltVal / 1024.0 * 3.3;
//     display_handler.clearDisplay();
//     display_handler.setCursor(0, 0);
//     display_handler.print("Reflectance: ");
//     display_handler.println(voltVal);
//     display_handler.println(voltVal/maxVolt * 100);
//     display_handler.display();
//     delay(500);
// }
