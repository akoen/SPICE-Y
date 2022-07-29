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
# define PWM_MOTOR_FWD_L PB6    
# define PWM_MOTOR_BACK_L PB7

# define PWM_MOTOR_FWD_R PB9
# define PWM_MOTOR_BACK_R PB8

# define PWM_FORMAT_MOTOR_FWD_L PB_6
# define PWM_FORMAT_MOTOR_BACK_L PB_7

# define PWM_FORMAT_MOTOR_FWD_R PB_9
# define PWM_FORMAT_MOTOR_BACK_R PB_8

// encoders
#define R_ENCODER_PIN1 PA11
// #define R_ENCODER_PIN2
#define L_ENCODER_PIN1 PA12
// #define L_ENCODER_PIN2 

// reflectance
# define REFLECTANCE_PIN_R PB1
# define REFLECTANCE_PIN_L PB0
# define REFLECTANCE_PIN_M PA3

# define REFLECTANCE_PIN_SIDE_L PA6    // placeholders
# define REFLECTANCE_PIN_SIDE_R PA7

// sonar
# define SONAR_TRIG_PIN_ALL PB14

# define SONAR_ECHO_PIN_L PB5
# define SONAR_ECHO_PIN_R PB4
# define SONAR_ECHO_PIN_F PA15

// claw-arm
# define CLAW_SERVO_PIN PA8
# define CLAW_SERVO_PIN_PWM_FORMAT PA_8
# define BRIDGE_SERVO_PIN PA9
# define BRIDGE_SERVO_PIN_PWM_FORMAT PA_9
# define ARM_SERVO_PIN PA10
# define ARM_SERVO_PIN_PWM_FORMAT PA_10
# define BOX_SERVO_PIN PA2
# define BOX_SERVO_PIN_PWM_FORMAT PA_2

// hall effect
# define MAGNETIC_SENSOR_PIN PB15

#endif