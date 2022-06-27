#include "Arduino.h"

#define LED_BUILTIN PC13
#define SONAR_TRIG PB11
#define SONAR_ECHO PB10

HardwareTimer *HB_TIM = new HardwareTimer(TIM1); //led blink timer
volatile bool led_state = false;

void Heartbeat()
{
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); //if led_state == true, go high, else, go low
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);

    Serial.begin(9600);
    Serial.println("Serial comms up");

    HB_TIM->setOverflow(5, HERTZ_FORMAT); //5hz blink
    HB_TIM->attachInterrupt(Heartbeat);
    HB_TIM->resume(); //start timer

}

void loop()
{ 
  //trigger sonar
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

/*
  uint32_t echo_time = pulseIn(SONAR_ECHO, HIGH); //measures duration of high pulse on echo pin
  float distance_cm = (echo_time/2) / 29.1;

  Serial.println(distance_cm);

*/
  //delay(60);

}