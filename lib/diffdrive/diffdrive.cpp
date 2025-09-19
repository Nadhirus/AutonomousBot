#include "diffdrive.h"
#include "pid.h"
#include "robot.h"
#include <Arduino.h>   // for millis(), types like int16_t, etc.
#include <math.h>      // for sinf(), cosf()
#include <Wire.h>

// Robot geometry
static const float WHEEL_RADIUS = 0.019f;   // 19 mm radius
static const float WHEEL_BASE   = 0.09f;    // 90 mm between wheels
static const int COUNTS_PER_REV = 909;      // encoder counts per wheel rev

static float v_lin_set = 0.0f, v_rot_set = 0.0f;
static PID_t pid_left, pid_right;

void diffdrive_init(void)
{
    // PID gains need tuning
    pid_init(&pid_left, 1.0f, 0.5f, 0.0f, -400, 400);
    pid_init(&pid_right, 1.0f, 0.5f, 0.0f, -400, 400);
}

void diffdrive_set_velocity(float v_lin, float v_rot)
{
    v_lin_set = v_lin;
    v_rot_set = v_rot;
}

void diffdrive_update(float dt)
{
    // Convert v_lin & v_rot to left/right wheel speeds (m/s)
    float v_left_set  = v_lin_set - (WHEEL_BASE/2.0f)*v_rot_set;
    float v_right_set = v_lin_set + (WHEEL_BASE/2.0f)*v_rot_set;

    // Get encoder deltas
    static int16_t prev_left = 0, prev_right = 0;
    int16_t left_counts = encoders.getCountsLeft();
    int16_t right_counts = encoders.getCountsRight();

    int16_t d_left = left_counts - prev_left;
    int16_t d_right = right_counts - prev_right;
    prev_left = left_counts;
    prev_right = right_counts;

    // counts to rad/s
    float wheel_angle_left  = (float)d_left / COUNTS_PER_REV * 2.0f * 3.14159f;
    float wheel_angle_right = (float)d_right / COUNTS_PER_REV * 2.0f * 3.14159f;

    float omega_left  = wheel_angle_left / dt;
    float omega_right = wheel_angle_right / dt;

    // Convert to linear wheel speeds
    float v_left_meas  = omega_left * WHEEL_RADIUS;
    float v_right_meas = omega_right * WHEEL_RADIUS;

    // PID to get PWM values
    float pwm_left  = pid_update(&pid_left, v_left_set, v_left_meas, dt);
    float pwm_right = pid_update(&pid_right, v_right_set, v_right_meas, dt);

    motors.setSpeeds((int)pwm_left, (int)pwm_right);
}
