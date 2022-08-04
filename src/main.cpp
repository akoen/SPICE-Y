#include <Arduino.h>

#define LED_BUILTIN PC13
#define ECHO_PIN PB7
#define TRIG_PIN PB6


#define SERIAL_BAUD 9600
#define TIMER1_OVERFLOW 200000 // microseconds
#define PWM_HZ 10

HardwareTimer *Timer1 = new HardwareTimer(TIM1);
volatile bool led_state = false;

void ISR_Heartbeat();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Timer1->setOverflow(TIMER1_OVERFLOW, MICROSEC_FORMAT);
  Timer1->attachInterrupt(ISR_Heartbeat);
  Timer1->resume();

  Serial.begin(SERIAL_BAUD);
}

void loop()
{
  // digitalWrite(TRIG_PIN, LOW);
  // delay(100);
  // digitalWrite(TRIG_PIN, HIGH);
  // delay(100);
  Serial.println(3);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  //recieve sonar reading
  uint32_t duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2 * 100; //*100 to conv m to cm

  // Serial.print("Distance (cm): "); 
  // Serial.println(distance);
  
}

void ISR_Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); // if led_state == true, go high, else, go low
}