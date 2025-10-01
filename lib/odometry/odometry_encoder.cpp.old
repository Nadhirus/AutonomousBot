#include "odometry.h"
#include <Arduino.h>   // for millis(), types like int16_t, etc.
#include <math.h>      // for sinf(), cosf()
#include "robot.h"

static Pose_t pose;
static int16_t prev_left = 0, prev_right = 0;

static const float WHEEL_RADIUS = 0.019f;   // 19 mm
static const float WHEEL_BASE   = 0.09f;    // 90 mm
static const int COUNTS_PER_REV = 909;

void odometry_init(void)
{
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.theta = 0.0f;
    prev_left = encoders.getCountsLeft();
    prev_right = encoders.getCountsRight();
}

void odometry_update(void)
{
    int16_t left_counts = encoders.getCountsLeft();
    int16_t right_counts = encoders.getCountsRight();

    int16_t d_left = left_counts - prev_left;
    int16_t d_right = right_counts - prev_right;

    prev_left = left_counts;
    prev_right = right_counts;

    float dl = (float)d_left / COUNTS_PER_REV * 2.0f * 3.14159f * WHEEL_RADIUS;
    float dr = (float)d_right / COUNTS_PER_REV * 2.0f * 3.14159f * WHEEL_RADIUS;

    float dc = (dl + dr) / 2.0f;
    float dtheta = (dr - dl) / WHEEL_BASE;

    pose.x += dc * cosf(pose.theta + dtheta/2.0f);
    pose.y += dc * sinf(pose.theta + dtheta/2.0f);
    pose.theta += dtheta;

    // Normalize theta
    if (pose.theta > 3.14159f) pose.theta -= 2.0f*3.14159f;
    if (pose.theta < -3.14159f) pose.theta += 2.0f*3.14159f;
}

Pose_t odometry_get_pose(void)
{
    return pose;
}
