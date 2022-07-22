#include "stm32f1xx_hal_adc.h"

#define BLUEPILL_CLOCK_SPEED 72
#define SERIAL_BAUD_RATE 115200

// OLED
#define OLED_SCREEN_WIDTH 128 // OLED display width, in pixels
#define OLED_SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

// IR
#define ADC_FAST_SAMPLETIME ADC_SAMPLETIME_71CYCLES_5 // 0.0001/(1/12e6×(71.5+12.5))/2 ~= 7 samples per 10 khz square wave
#define IR_SENS_NUM_READINGS 2048
#define IR_SMOOTHING_ALPHA 0.05
