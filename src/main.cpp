#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "robot.h"  

// -------------------- Hardware --------------------
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

// -------------------- Shared Variables --------------------
volatile float v_lin = 0.0f;   // m/s
volatile float v_rot = 0.0f;   // rad/s
SemaphoreHandle_t velMutex;

// PID update rate
const TickType_t updatePeriodTicks = pdMS_TO_TICKS(20); // 50 Hz

// -------------------- Task Prototypes --------------------
void vTaskDiffDriveUpdate(void *pvParameters);
void vTaskDummyDrive(void *pvParameters);

// -------------------- Setup --------------------
void setup() 
{
    // Wait for button press and indicate start
    buttonA.waitForButton();
    ledGreen(1);

    // Initialize diffdrive and PID
    diffdrive_init();

    // Create mutex for velocity (optional, one writer task here)
    velMutex = xSemaphoreCreateMutex();

    // High-priority PID task
    xTaskCreate(vTaskDiffDriveUpdate, "DiffUpdate", 384, NULL, 3, NULL);

    // Low-priority dummy driving task
    xTaskCreate(vTaskDummyDrive, "DummyDrive", 256, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop() 
{
    // Not used with FreeRTOS
}

// -------------------- PID Update Task --------------------
void vTaskDiffDriveUpdate(void *pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, updatePeriodTicks);
        float dt = (float)updatePeriodTicks / 1000.0f; // seconds

        // Run PID update
        diffdrive_update(dt);
    }
}

// -------------------- Dummy Drive Task --------------------
void vTaskDummyDrive(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t stepDelay = pdMS_TO_TICKS(2000); // 2 s per motion

    // Safe velocities for Zumo32U4
    const float speed_lin = 250;   // m/s
    const float speed_rot = 0;   // rad/s

    for (;;)
    {
        // Forward
        diffdrive_set_velocity(speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        // Backward
        diffdrive_set_velocity(-speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        // Turn left
        diffdrive_set_velocity(0.0f, speed_rot);
        vTaskDelay(stepDelay);

        // Turn right
        diffdrive_set_velocity(0.0f, -speed_rot);
        vTaskDelay(stepDelay);
    }
}
