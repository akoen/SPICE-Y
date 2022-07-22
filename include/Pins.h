#ifndef PinsFile
#define PinsFile

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
# define REFLECTANCE_PIN_M PA10
# define REFLECTANCE_PIN_R PA11
# define REFLECTANCE_PIN_L PA12

# define REFLECTANCE_PIN_SIDE_L 0    // placeholders
# define REFLECTANCE_PIN_SIDE_R 0

// sonar
# define SONAR_TRIG_PIN_L PB12
# define SONAR_TRIG_PIN_R PB13
# define SONAR_TRIG_PIN_F PB14

# define SONAR_ECHO_PIN_L PB15
# define SONAR_ECHO_PIN_R PA8
# define SONAR_ECHO_PIN_F PA9

// claw-arm
# define CLAW_SERVO_PIN 0
# define ARM_SERVO_PIN 0
# define BRIDGE_SERVO_PIN 0
# define BOX_SERVO_PIN 0

// hall effect
# define MAGNETIC_SENSOR_PIN 0

#endif
