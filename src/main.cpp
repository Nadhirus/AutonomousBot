#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "robot.h"  

#define MAX_SPEED 150

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

volatile float v_lin = 0.0f;
volatile float v_rot = 0.0f;

SemaphoreHandle_t velMutex;

const TickType_t updatePeriodTicks = pdMS_TO_TICKS(20); // 50 Hz update


void vTaskSetVelocity(void *pvParameters);
void vTaskDiffDriveUpdate(void *pvParameters);
void vTaskDummyDrive(void *pvParameters);

void setup()
{
  buttonA.waitForButton();
  (void)ledGreen(1);

  diffdrive_init();

  velMutex = xSemaphoreCreateMutex();

  xTaskCreate(vTaskSetVelocity, "SetVel", 128, NULL, 2, NULL);
  xTaskCreate(vTaskDiffDriveUpdate, "DiffUpdate", 256, NULL, 3, NULL);
  xTaskCreate(vTaskDummyDrive, "DummyDrive", 128, NULL, 1, NULL);


  vTaskStartScheduler();
}

void loop()
{
}

void vTaskSetVelocity(void *pvParameters)
{
  (void) pvParameters;
  float local_vlin = 0.2f;  // m/s
  float local_vrot = 0.0f;  // rad/s

  for(;;)
  {
    // lock the mutex before changing shared variables
    if (xSemaphoreTake(velMutex, portMAX_DELAY) == pdTRUE)
    {
      v_lin = local_vlin;
      v_rot = local_vrot;
      xSemaphoreGive(velMutex);
    }

    // Push to diffdrive (thread-safe way)
    diffdrive_set_velocity(local_vlin, local_vrot);

    // Change velocities every 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));
    local_vlin = -local_vlin; // reverse direction
  }
}

/*Constantly run PID update with diffdrive_update() */
void vTaskDiffDriveUpdate(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, updatePeriodTicks);

    float dt = (float)updatePeriodTicks / 1000.0f; // seconds

    diffdrive_update(dt);
  }
}


void vTaskDummyDrive(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t stepDelay = pdMS_TO_TICKS(2000); // 2 seconds per motion

    // velocities in m/s and rad/s
    const float speed_lin = 0.5f;
    const float speed_rot = 10.0f; // rad/s

    for (;;)
    {
        // Move forward
        diffdrive_set_velocity(speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        // Move backward
        diffdrive_set_velocity(-speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        // Turn left in place
        diffdrive_set_velocity(0.0f, speed_rot);
        vTaskDelay(stepDelay);

        // Turn right in place
        diffdrive_set_velocity(0.0f, -speed_rot);
        vTaskDelay(stepDelay);
    }
}
