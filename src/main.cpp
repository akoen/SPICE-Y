//-------------------------------------------
// Lib Include Files
//-------------------------------------------

#include <Arduino.h>
#include "SpiceServo.h"
#include "Sonar.h"

//-------------------------------------------
// Constant Definitions & Macros
//-------------------------------------------

#define LED_BUILTIN PC13
#define SERIAL_BAUD 9600
#define TIMER1_OVERFLOW 200000 // microseconds

//-------------------------------------------
// Local Variables
//-------------------------------------------

typedef enum
{
  CHANNEL1 = 1,
  CHANNEL2 = 2,
  CHANNEL3 = 3,
  CHANNEL4 = 4
} timerChannels_t;

timerChannels_t channel1 = CHANNEL1;
timerChannels_t channel2 = CHANNEL2;
timerChannels_t channel3 = CHANNEL3;
timerChannels_t channel4 = CHANNEL4;

HardwareTimer *Timer1 = new HardwareTimer(TIM1);
volatile bool led_state = false;
double newDistance = 0;
double previousDistance = 0;

//-------------------------------------------
// Local Function Prototypes
//-------------------------------------------

void setup();
void ISR_Heartbeat();

//-------------------------------------------
// Local Function Implementations
//-------------------------------------------

void setup()
{
  // pin modes and interrupts
  pinMode(LED_BUILTIN, OUTPUT);
  setupSonar();

  // harware timers
  Timer1->setOverflow(TIMER1_OVERFLOW, MICROSEC_FORMAT);
  Timer1->attachInterrupt(ISR_Heartbeat);                                                                         // full complete callback                                                                   // TODO: investigate whether this interfere with GPIO on this pin at all?
  Timer1->setCaptureCompare(channel1, (SONAR_MEAS_DELAY * 1000 / TIMER1_OVERFLOW) * 100, PERCENT_COMPARE_FORMAT); // 35% complete callback (sonar reading every 70 milliseconds)
  Timer1->attachInterrupt(channel1, ISR_SonarTrigger);
  Timer1->resume();

  // serial
  Serial.begin(SERIAL_BAUD);
}

void loop()
{
  if (readyToTrigger)
  { // set every 70mils by channel associated with hardware timer 1
    TriggerSonar();
  }

  if (pulseCaptured)
  {
    pulseCaptured = false;
    newDistance = (pulseDuration / 2) * SPEED_SOUND_MICS; // centimeters
    if (newDistance != previousDistance)
    {
      Serial.println(newDistance);
      previousDistance = newDistance;
    }
  }

  // Serial.println("Loop");
}

void ISR_Heartbeat()
{
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW); // if led_state == true, go high, else, go low
}