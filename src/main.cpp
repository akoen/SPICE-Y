#include "main.h"

//Globals
Servo clawServo;
Servo armServo;
HardwareTimer *HB_TIM = new HardwareTimer(TIM1); //led blink timer
volatile bool led_state = false;

void setup()
{
    //pin setups
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    //timer setup
    HB_TIM->setOverflow(5, HERTZ_FORMAT); //5hz blink
    HB_TIM->attachInterrupt(Heartbeat);
    HB_TIM->resume(); //start timer
    
    //serial setup
    Serial.begin(9600);
    Serial.println("Serial comms up");

}

void loop()
{ 
  Serial.println(*pDistance);
}

void Heartbeat()
{
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); //if led_state == true, go high, else, go low
}