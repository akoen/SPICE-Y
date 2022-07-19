#include "SpiceServo.h"

/*Purpose: Attach pwm pins to servo objects
  Arguments: None
  Returns: None
*/
void setupServo()
{
  Servo *clawServo = new Servo();
  Servo *armServo = new Servo();

  clawServo->attach(CLAW_SERV_PIN);
  armServo->attach(ARM_SERV_PIN);
}

/*Purpose: Send a pwm position signal to a Servo object
  Arguments:
    servo: Servo object
    position: 0 to 180 (degrees)
  Returns: None
*/
void sendServoSignal(Servo servo, uint8_t position)
{
  servo.write(position);
}