#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include "robot.h"

// Shared state (pose)
static Pose_t currentPose;
static SemaphoreHandle_t poseMutex; // protects currentPose

// Task: handles low-level control loop (diffdrive + odometry)
void TaskControl(void *pvParameters) {
    (void)pvParameters;

    unsigned long lastUpdate = millis();

    for (;;) {
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        lastUpdate = now;

        // Command example: 0.1 m/s forward, 0 rad/s rotation
        diffdrive_set_velocity(0.1f, 0.0f);
        diffdrive_update(dt);

        // Update odometry
        odometry_update();

        // Copy odometry pose into shared state safely
        Pose_t p = odometry_get_pose();
        xSemaphoreTake(poseMutex, portMAX_DELAY);
        currentPose = p;
        xSemaphoreGive(poseMutex);

        vTaskDelay(pdMS_TO_TICKS(10)); // run at 100 Hz
    }
}

// Task: high-level logic (reads pose safely)
void TaskLogic(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        Pose_t p;

        // safely read pose
        xSemaphoreTake(poseMutex, portMAX_DELAY);
        p = currentPose;
        xSemaphoreGive(poseMutex);

        // Example: use p.x, p.y, p.theta for decision making
        // e.g., wall follower, mapping, etc.

        vTaskDelay(pdMS_TO_TICKS(50)); // run at 20 Hz
    }
}

void setup() {
    robot_init();

    // Create mutex for pose
    poseMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreate(TaskControl, "Ctrl", 256, NULL, 3, NULL);
    xTaskCreate(TaskLogic, "Logic", 256, NULL, 2, NULL);

    // Start scheduler
    vTaskStartScheduler();
}

void loop() {
    // never called in FreeRTOS
}
