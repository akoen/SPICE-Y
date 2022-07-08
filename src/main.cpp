#include "Arduino.h"

// Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PC13
#define PWM_FWD PA_3
#define PWM_REV PA_7
#define DUTY_CYCLE_ADJ PB1
#define DIRECTION_CTRL PB10

#define FULL_SCALE_ADC 1024
#define FULL_SCALE_PWM 65535
#define PWM_FREQ 10

HardwareTimer *HB_TIM = new HardwareTimer(TIM1);
volatile bool led_state = false;
float duty_cycle_percent = 0.5; //duty cycle fraction
float pwm_duty_cycle = FULL_SCALE_PWM * duty_cycle_percent;

void Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM_FWD, OUTPUT);
  pinMode(PWM_REV, OUTPUT);

  /*
  //Setup LED Heartbeat hardware timer
  heartbeat_timer.setOverflow(1, HERTZ_FORMAT); //set frequency of timer
  heartbeat_timer.attachInterrupt(Heartbeat);
  heartbeat_timer.resume();
  */

  HB_TIM->setOverflow(5, HERTZ_FORMAT);
  HB_TIM->attachInterrupt(Heartbeat);
  HB_TIM->resume();

  //starting pwm once
  pwm_start(PWM_REV, PWM_FREQ, pwm_duty_cycle, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  
}

void loop()
{
}