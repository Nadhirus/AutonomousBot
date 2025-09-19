#include "pid.h"

void pid_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;
    return output;
}
