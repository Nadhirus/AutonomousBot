#include <Wire.h>
#include <Zumo32U4.h>
#include "robot.h"

#define MAX_SPEED 400

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

void setup()
{
  // this is to start the program
  buttonA.waitForButton();
  (void)ledGreen(1);
  diffdrive_init();
  delay(1000);
}

void loop()
{

  diffdrive_set_velocity(250, 0); // m/s, rad/s
  delay(50);
  diffdrive_update(50);       // run PID to achieve setpoint

  delay(500);
}
