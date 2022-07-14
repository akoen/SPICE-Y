// /**
//  * This file is responsible for all encoder purposes, used for position sensing.
//  */

// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
// #include <math.h>

// // OLED standard setup
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET 	-1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// // pins
// #define R_ENCODER_PIN1 PB10
// #define R_ENCODER_PIN2 PB11 
// #define L_ENCODER_PIN1 PB14
// #define L_ENCODER_PIN2 PB15 



// // const
// const double maxVolt = 3.3;
// const double pulseWidth = 1389.9185;
// const double wheelDiameter = 6.4; // cm
// // vars
// volatile uint32_t interruptCountLW = 0;
// volatile uint32_t interruptCountRW = 0;

// volatile long revLW = 0;
// volatile int dirLW = 0;
// volatile double posLW = 0;

// volatile long revRW = 0;
// volatile int dirRW = 0;
// volatile double posRW = 0;

// void checkPosLW(int pin1, int pin2) {
//     int state1 = digitalRead(pin1);
//     int state2 = digitalRead(pin2);
//     if (state1 == 0) {
//         revLW = -99999; // shouldn't occur
//         dirLW = -1;
//     }
//     if (state2 == 0) {  // CW (my def)
//         revLW--;
//         dirLW = false;
//     } else {    // CCW
//         revLW++;
//         dirLW = true;
//     } 
//     interruptCountLW++;
// }

// void checkPosRW(int pin1, int pin2) {
//     int state1 = digitalRead(pin1);
//     int state2 = digitalRead(pin2);
//     if (state1 == 0) {
//         dirRW = -99999; // shouldn't occur
//         dirRW = -1;
//     }
//     if (state2 == 0) {  // CW (my def)
//         revRW--;
//         dirRW = false;
//     } else {    // CCW
//         revRW++;
//         dirRW = true;
//     } 
//     interruptCountRW++;
// }

// void ISR_LeftWheel() {
//     checkPosLW(L_ENCODER_PIN1, L_ENCODER_PIN2);
// }

// void ISR_RightWheel() {
//     checkPosRW(R_ENCODER_PIN1, R_ENCODER_PIN2);
// }

// /**
//  * @brief Sets up the OLED display
//  */;
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
//     // set analog resolution to 12 bits, as Arduino lib decreases to 10
//     // analogReadResolution(12);

    // attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN1), ISR_LeftWheel, RISING); // ISR called when pin signal changes 0->1
//     attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN1), ISR_RightWheel, RISING); // ISR called when pin signal changes 0->1
//     // attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN2), checkPos, CHANGE);
    
//     OLEDSetup();
// }

// /**
//  * @brief Read the value of the analog-in pin and set the timing for the LED
//  *    	blink period depending on this value.
//  * @param none
//  * @retval none
//  */
// void loop() {
//     posLW = (revLW / pulseWidth)*PI*wheelDiameter;
//     posRW = (revRW / pulseWidth)*PI*wheelDiameter;
//     display_handler.clearDisplay();
//     display_handler.setCursor(0, 0);
//     display_handler.println("Pos(L,R):  ");
//     display_handler.print(posLW);
//     display_handler.print(", ");
//     display_handler.println(posRW);
//     display_handler.println();

//     display_handler.println("Dir(L,R):  ");
//     display_handler.print(dirLW);
//     display_handler.print(", ");
//     display_handler.println(dirRW);  
//     display_handler.println();
    
//     display_handler.println("ISR(L,R):  ");
//     display_handler.print(interruptCountLW);
//     display_handler.print(", ");
//     display_handler.println(interruptCountRW);    

//     display_handler.display();
// }
