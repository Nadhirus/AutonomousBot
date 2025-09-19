#ifndef DIFFDRIVE_H
#define DIFFDRIVE_H

void diffdrive_init(void);
void diffdrive_set_velocity(float v_lin, float v_rot); // m/s, rad/s
void diffdrive_update(float dt); // run PID to achieve setpoint

#endif
