#include "odometry.h"
#include <Arduino.h>
#include <math.h>
#include "robot.h"
#include "Kalman.h"

static Pose_t pose;
static int16_t prev_left = 0, prev_right = 0;

static const float WHEEL_RADIUS = 0.019f;   // 19 mm
static const float WHEEL_BASE   = 0.09f;    // 90 mm
static const int COUNTS_PER_REV = 909;

static Kalman kalman;
static unsigned long last_update = 0;
static float gyroBiasZ = 0.0f;

void odometry_init(void)
{
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.theta = 0.0f;
    prev_left = encoders.getCountsLeft();
    prev_right = encoders.getCountsRight();

    kalman.setAngle(0.0f);
    kalman.setQangle(0.001f);
    kalman.setQbias(0.003f);
    kalman.setRmeasure(0.03f);

    // Compute gyro bias over 50 readings
    int32_t sum = 0;
    for (int i = 0; i < 50; i++) {
        imu.read();
        sum += imu.g.z;
        delay(5);
    }
    gyroBiasZ = sum / 50.0f;

    last_update = millis();
}

void odometry_update(void)
{
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0f; // seconds
    last_update = now;

    // Read encoder deltas
    int16_t left_counts = encoders.getCountsLeft();
    int16_t right_counts = encoders.getCountsRight();
    int16_t d_left = left_counts - prev_left;
    int16_t d_right = right_counts - prev_right;
    prev_left = left_counts;
    prev_right = right_counts;

    float dl = (float)d_left / COUNTS_PER_REV * 2.0f * 3.14159f * WHEEL_RADIUS;
    float dr = (float)d_right / COUNTS_PER_REV * 2.0f * 3.14159f * WHEEL_RADIUS;
    float dc = (dl + dr) / 2.0f;

    imu.read();

    // Use Z-axis gyro for yaw rate
    float gyro_rate_dps = 0.0f;
    if (imu.getType() == Zumo32U4IMUType::LSM6DS33_LIS3MDL) {
        gyro_rate_dps = (imu.g.z - gyroBiasZ) * 0.00875f;  // LSM6DS33 ±245 dps
    } else {
        gyro_rate_dps = (imu.g.z - gyroBiasZ) / 131.0f;    // L3GD20H ±250 dps
    }

    // Convert gyro delta to radians
    float dtheta = gyro_rate_dps * dt * 3.14159f / 180.0f;

    // Update heading
    pose.theta += dtheta;

    // Keep theta in [-π, π]
    if (pose.theta > 3.14159f) pose.theta -= 2.0f * 3.14159f;
    if (pose.theta < -3.14159f) pose.theta += 2.0f * 3.14159f;

    // Update position in robot frame
    pose.x += dc * cosf(pose.theta);
    pose.y += dc * sinf(pose.theta);
}

Pose_t odometry_get_pose(void)
{
    return pose;
}
