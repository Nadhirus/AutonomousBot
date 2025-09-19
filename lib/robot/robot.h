#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>   


// Pololu hardware objects
extern Zumo32U4Motors motors;
extern Zumo32U4Encoders encoders;
extern Zumo32U4IMU imu;
extern Zumo32U4ProximitySensors proximitySensors;
extern Zumo32U4LineSensors lineSensors;
extern Zumo32U4Buzzer buzzer;
extern Zumo32U4ButtonA buttonA;
extern Zumo32U4ButtonB buttonB;
extern Zumo32U4ButtonC buttonC;

// Your modules
#include "pid.h"
#include "diffdrive.h"
#include "odometry.h"
#include "occupancy_grid.h"
#include "wallfollower.h"

// High-level initialization
void robot_init(void);

#endif
