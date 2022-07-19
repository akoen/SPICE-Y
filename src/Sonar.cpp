#include "Sonar.h"
#include <Arduino.h>

void setupSonar(){
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}


//to be called in interrupt associated with hardware timer (every 70 milliseconds)
void getDistance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH); //duration of high pulse proportional to distance sound waves travelled
  double distance = (duration*0.0343)/2.0; //distance in cm d = speed of sound in cm/s times time (duration) divide 2 (round trip)
  pDistance = &distance;
  

}

