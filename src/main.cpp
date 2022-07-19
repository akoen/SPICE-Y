#include "main.h"

// Globals
HardwareTimer *Timer1 = new HardwareTimer(TIM1);
volatile bool led_state = false;

void setup()
{
  // pin setup
  pinMode(LED_BUILTIN, OUTPUT);
  setupSonar();

  // timer setup
  Timer1->setOverflow(TIMER1_OVERFLOW, MICROSEC_FORMAT);
  Timer1->attachInterrupt(ISR_Heartbeat);                                                                           // full complete callback
  Timer1->setMode(channel1, TIMER_DISABLED);                                                                        // TODO: investigate whether this interfere with GPIO on this pin at all?
  Timer1->setCaptureCompare(channel1, ((SONAR_MEAS_DELAY * 1000) / TIMER1_OVERFLOW) * 100, PERCENT_COMPARE_FORMAT); // 35% complete callback (sonar reading every 70milliseconds)
  Timer1->attachInterrupt(channel1, ISR_GetDistance);
  Timer1->resume();

  // serial setup
  Serial.begin(SERIAL_BAUD);
  Serial.println("Serial comms up");
}

void loop()
{
  Serial.println(*pDistance);
}

void ISR_Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); // if led_state == true, go high, else, go low
}