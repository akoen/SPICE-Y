/**
* @brief Blink: Turns on an LED for one second then off for
* one second and then repeats.
*/

#include "Arduino.h"

//Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PC13
#define PWM_PIN PB6
#define TAPE_SNS PA7
#define IR_SNS PA6
#define PULSE_SONAR PB11
#define SONAR_SNS PB0
#define PWM_FWD PA_3
#define PWM_BACK PA_7

#define PWM_FREQ 10

/**
 * @brief Initialize LED pin as digital write.
 * @param none
 * @retval none
 */

void setup()
{
  //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PA7, INPUT); //analog input
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PULSE_SONAR, OUTPUT);
  pinMode(SONAR_SNS, INPUT);
  pinMode(PWM_FWD, OUTPUT);
  pinMode(PWM_BACK, OUTPUT);
  Serial.begin(9600);
  Serial.println("Serial comms up");

  //Serial.println("Serial comms up");

  //digitalWrite(PWM_PIN, HIGH);
  //digitalWrite(PULSE_SONAR, HIGH);

}

/**
 * @brief Turn LED on for 1 sec and off for 1 sec.
 * @param none
 * @retval none
 */
void loop()
{ 

  /*16 bit format for duty cycle, 2^16 = 65535*/
  //pwm_start(PWM_FWD, PWM_FREQ, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  /*
  int duty_cycle_percent = 75;
  float duty_cycle_fn = 65535 * (duty_cycle_percent / 100); //full scale
  pwm_start(PWM_BACK, PWM_FREQ, 65535/2, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  */
  pwm_start(PWM_FWD, PWM_FREQ, 65535/2, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  

  uint32_t delay_ms = 1000;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);


  
  //Reflectance (Tape) Analog Read
  
  /*
  float reflectance_sns = analogRead(TAPE_SNS);
  Serial.println(reflectance_sns);
  */


  //Pulse Infrared Beacon
  //Infrared Beacon Analog Read

  //float ir_sns = analogRead(IR_SNS);
  //Serial.println(ir_sns);

  //Pulse and Read Sonar Sensor
  
  //digitalWrite(PULSE_SONAR, LOW); //turn fet off, pull trig to 5V
  //delayMicroseconds(10);
  //digitalWrite(PULSE_SONAR, HIGH); //turn fet on, ground trig (end of pulse)
  //delay(100);

}









