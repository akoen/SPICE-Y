#ifndef CONFIG
#define CONFIG

// #include "stm32f1xx_hal_adc.h"
#include <Arduino.h>

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

/*Pins*/

// motors
# define PWM_MOTOR_FWD_L PB1
# define PWM_MOTOR_BACK_L PB0

# define PWM_MOTOR_FWD_R PA6
# define PWM_MOTOR_BACK_R PA7

# define PWM_FORMAT_MOTOR_FWD_L PB_1
# define PWM_FORMAT_MOTOR_BACK_L PB_0

# define PWM_FORMAT_MOTOR_FWD_R PA_6
# define PWM_FORMAT_MOTOR_BACK_R PA_7

// encoders
#define R_ENCODER_PIN1 PB10
// #define R_ENCODER_PIN2 PB14
#define L_ENCODER_PIN1 PB11
// #define L_ENCODER_PIN2 PB15 

// reflectance
# define REFLECTANCE_PIN_M PA8
# define REFLECTANCE_PIN_R PA9
# define REFLECTANCE_PIN_L PA10

# define REFLECTANCE_PIN_SIDE_L 0    // placeholders
# define REFLECTANCE_PIN_SIDE_R 0

// sonar
# define SONAR_TRIG_PIN_L PB12
# define SONAR_TRIG_PIN_R PA4
# define SONAR_TRIG_PIN_F PB14

# define SONAR_ECHO_PIN_L PB15
# define SONAR_ECHO_PIN_R PA3
# define SONAR_ECHO_PIN_F PA9

// claw-arm
# define CLAW_SERVO_PIN 0
# define ARM_SERVO_PIN 0
# define BRIDGE_SERVO_PIN 0
# define BOX_SERVO_PIN 0

// hall effect
# define MAGNETIC_SENSOR_PIN 0

#endif