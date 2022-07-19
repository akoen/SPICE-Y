#include "Sonar.h"
#include <Arduino.h>

volatile bool readyToTrigger = false;
volatile bool pulseCaptured = false;
volatile uint32_t pulseDuration = 0;

//called in setup() in main.cpp
void setupSonar()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(ECHO_PIN, ISR_SonarEcho, CHANGE); //interrupt triggered when pin high then when pin goes low
}

//send sequence of pulses to sonar trigger line. Not to be used in any interrupts
void TriggerSonar(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  readyToTrigger = false;
}


//called every 70 milliseconds in channel 1 TIM1 associated interrupt
void ISR_SonarTrigger()
{
  readyToTrigger = true;
}

//called in response to echoPin rising or falling edge
void ISR_SonarEcho(){
  static uint32_t startTime;
  if (digitalRead(ECHO_PIN)){ //pin high means start of echo (return) pulse
    startTime = micros();
  }
  else{ //interrupt only triggered on change (rising or falling). If not rising, must be falling (echo done)
    pulseDuration = micros() - startTime;
    pulseCaptured = true;
  }
}