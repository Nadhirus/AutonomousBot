#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
    float out_min, out_max;
} PID_t;

void pid_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float pid_update(PID_t *pid, float setpoint, float measurement, float dt);

#endif
