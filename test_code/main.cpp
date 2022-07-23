#include <Arduino.h>

#define LED_BUILTIN PC13
#define RIGHT_FWD_PIN PB_6
#define RIGHT_REV_PIN PB_7


#define SERIAL_BAUD 9600
#define TIMER1_OVERFLOW 200000 // microseconds
#define PWM_HZ 10
#define MAX_DUTY 65535

HardwareTimer *Timer1 = new HardwareTimer(TIM1);
volatile bool led_state = false;

void ISR_Heartbeat();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Timer1->setOverflow(TIMER1_OVERFLOW, MICROSEC_FORMAT);
  Timer1->attachInterrupt(ISR_Heartbeat);
  Timer1->resume();

  Serial.begin(SERIAL_BAUD);
}

void loop()
{
  pwm_start(RIGHT_FWD_PIN, PWM_HZ, MAX_DUTY/2, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  delay(5000);
  pwm_stop(RIGHT_FWD_PIN);
  pwm_start(RIGHT_REV_PIN, PWM_HZ, MAX_DUTY/2, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);

}

void ISR_Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); // if led_state == true, go high, else, go low
}