#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "robot.h"

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

volatile float v_lin = 0.0f;
volatile float v_rot = 0.0f;
SemaphoreHandle_t velMutex;

const TickType_t updatePeriodTicks = pdMS_TO_TICKS(20);

Zumo32U4LCD lcd;

void vTaskDiffDriveUpdate(void *pvParameters);
void vTaskDummyDrive(void *pvParameters);
void vTaskDisplayOdometry(void *pvParameters);

void setup() 
{
    buttonA.waitForButton();
    ledGreen(1);

    diffdrive_init();
    odometry_init();
    lcd.init();

    velMutex = xSemaphoreCreateMutex();

    xTaskCreate(vTaskDiffDriveUpdate, "DiffUpdate", 384, NULL, 3, NULL);
    xTaskCreate(vTaskDummyDrive, "DummyDrive", 256, NULL, 1, NULL);
    xTaskCreate(vTaskDisplayOdometry, "DisplayOdo", 256, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() 
{
}

void vTaskDiffDriveUpdate(void *pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, updatePeriodTicks);
        float dt = (float)updatePeriodTicks / 1000.0f;
        diffdrive_update(dt);
    }
}

void vTaskDummyDrive(void *pvParameters)
{
    (void) pvParameters;
    const TickType_t stepDelay = pdMS_TO_TICKS(2000);
    const float speed_lin = 100;
    const float speed_rot = 0;

    for (;;)
    {
        diffdrive_set_velocity(speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        diffdrive_set_velocity(-speed_lin, 0.0f);
        vTaskDelay(stepDelay);

        diffdrive_set_velocity(0.0f, speed_rot);
        vTaskDelay(stepDelay);

        diffdrive_set_velocity(0.0f, -speed_rot);
        vTaskDelay(stepDelay);
    }
}

void vTaskDisplayOdometry(void *pvParameters)
{
    (void) pvParameters;
    const TickType_t displayRate = pdMS_TO_TICKS(200);

    for (;;)
    {
        odometry_update();
        Pose_t pose = odometry_get_pose();
        int x_cm = (int)(pose.x * 100.0f);
        int y_cm = (int)(pose.y * 100.0f);
        int theta_deg = (int)(pose.theta * 180.0f / 3.14159f);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("(");
        lcd.print(x_cm);
        lcd.print(",");
        lcd.print(y_cm);
        lcd.print(")");

        lcd.setCursor(0, 1);
        lcd.print(theta_deg);
        lcd.print((char)223);

        vTaskDelay(displayRate);
    }
}
