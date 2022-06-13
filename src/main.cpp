#include "Arduino.h"

// Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PC13
#define PWM_FWD PA_3
#define PWM_REV PA_7
#define DUTY_CYCLE_ADJ PB1
#define DIRECTION_CTRL PB10

#define FULL_SCALE_ADC 1024
#define FULL_SCALE_PWM 65535
#define PWM_FREQ 100

#define FWD 1
#define REV 0

HardwareTimer *HB_TIM = new HardwareTimer(TIM1);
HardwareTimer *M1_PWM_TIM = new HardwareTimer(TIM2);
volatile bool led_state = false;

void Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
}

void testMotor()
{
}

void UpdateMotor1State()
{

  // call this function every x times a second - how many?

  int direction = digitalRead(DIRECTION_CTRL);

  /* uncomment once you have a pot hooked up to this adc channel
  float duty_cycle_percent = analogRead(DUTY_CYCLE_ADJ) / FULL_SCALE_ADC; //number between 0 - 1023
  float pwm_duty_cycle = duty_cycle_percent * FULL_SCALE_PWM;
  */

  // comment this out when you've hooked up adc channel pot duty cycle control
  float pwm_duty_cycle = FULL_SCALE_PWM * 0.5;

  switch (direction)
  {
  case FWD:
    pwm_start(PWM_FWD, PWM_FREQ, pwm_duty_cycle, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    break;

  case REV:
    pwm_start(PWM_REV, PWM_FREQ, pwm_duty_cycle, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    break;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM_FWD, OUTPUT);
  pinMode(PWM_REV, OUTPUT);
  pinMode(DUTY_CYCLE_ADJ, INPUT);
  pinMode(DIRECTION_CTRL, INPUT);

  /*
  //Setup LED Heartbeat hardware timer
  heartbeat_timer.setOverflow(1, HERTZ_FORMAT); //set frequency of timer
  heartbeat_timer.attachInterrupt(Heartbeat);
  heartbeat_timer.resume();
  */

  HB_TIM->setOverflow(5, HERTZ_FORMAT);
  HB_TIM->attachInterrupt(Heartbeat);
  HB_TIM->resume();

  M1_PWM_TIM->setOverflow(10, HERTZ_FORMAT);
  M1_PWM_TIM->attachInterrupt(testMotor);
  M1_PWM_TIM->resume();

  /*
  //Setup MOTOR1 hardware timer
  motor1_pwm_timer.setOverflow(25, HERTZ_FORMAT); //how often motor can sample changes
  motor1_pwm_timer.attachInterrupt(UpdateMotor1State);
  motor1_pwm_timer.resume();
  */
}

void loop()
{
}