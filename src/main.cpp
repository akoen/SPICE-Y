#include <Arduino.h>

#define LED_BUILTIN PC13
#define REFLECTANCE_PIN PA0


#define SERIAL_BAUD 9600
#define TIMER1_OVERFLOW 200000 // microseconds
#define PWM_HZ 10

HardwareTimer *Timer1 = new HardwareTimer(TIM1);
volatile bool led_state = false;

void ISR_Heartbeat();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REFLECTANCE_PIN, INPUT);

  Timer1->setOverflow(TIMER1_OVERFLOW, MICROSEC_FORMAT);
  Timer1->attachInterrupt(ISR_Heartbeat);
  Timer1->resume();

  Serial.begin(SERIAL_BAUD);
}

void loop()
{
  int laser_reflectance_analog = analogRead(REFLECTANCE_PIN); //higher analog value = less reflectance = larger "distance" from laser to object
  Serial.print("ADC Reading: "); 
  Serial.println(laser_reflectance_analog);
  delay(500);
}

void ISR_Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); // if led_state == true, go high, else, go low
}