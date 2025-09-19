#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct {
    float x;     // m
    float y;     // m
    float theta; // rad
} Pose_t;

void odometry_init(void);
void odometry_update(void);
Pose_t odometry_get_pose(void);

#endif
