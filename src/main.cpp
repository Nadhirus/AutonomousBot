#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "robot.h"

// Robot peripherals
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4IMU imu;

// global variables (NEED TO ADD CONCURRENCY GUARDRAILS!)
volatile float v_lin = 0.0f;
volatile float v_rot = 0.0f;
SemaphoreHandle_t velMutex;
char report[120];

// FreeRTOS stuff
const TickType_t updatePeriodTicks = pdMS_TO_TICKS(20);

// Different task functions
void vTaskDriveMotors(void *pvParameters);
void vTaskNavigation(void *pvParameters);
void vTaskDisplay(void *pvParameters);
void vTaskOdometry(void *pvParameters);

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    diffdrive_init();
    odometry_init();
    lcd.init();

    buttonA.waitForButton();
    ledGreen(1);

    if (!imu.init())
    {
        ledRed(1);
        while (1)
        {
            Serial.println(F("Failed to initialize IMU sensors."));
            delay(100);
        }
    }

    imu.enableDefault();

    velMutex = xSemaphoreCreateMutex();

    xTaskCreate(vTaskDriveMotors, "DriveMotors", 192, NULL, 3, NULL); // 192
    xTaskCreate(vTaskNavigation, "Navigation", 128, NULL, 1, NULL);
    xTaskCreate(vTaskOdometry, "Odometry", 384, NULL, 2, NULL); 
    xTaskCreate(vTaskDisplay, "Display", 128, NULL, 1, NULL); 

    vTaskStartScheduler();
}

void loop()
{
}

void vTaskDriveMotors(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, updatePeriodTicks);
        float dt = (float)updatePeriodTicks / 1000.0f;
        diffdrive_update(dt);
    }
}

void vTaskNavigation(void *pvParameters)
{
    (void)pvParameters;
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

void vTaskDisplay(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t displayRate = pdMS_TO_TICKS(200); // 5 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    int last_x = 0x7FFF, last_y = 0x7FFF, last_theta = 0x7FFF;

    for (;;)
    {
        Pose_t pose = odometry_get_pose();

        // Convert to cm
        int x_cm = (int)(pose.x * 100.0f);
        int y_cm = (int)(pose.y * 100.0f);

        // Convert theta to degrees and wrap to [-180, +180]
        int theta_deg = (int)(pose.theta * 180.0f / 3.14159f);
        while (theta_deg > 180) theta_deg -= 360;
        while (theta_deg < -180) theta_deg += 360;

        // Update position only if changed
        if (x_cm != last_x || y_cm != last_y)
        {
            lcd.setCursor(0, 0);
            lcd.print("(");
            lcd.print(x_cm);
            lcd.print(",");
            lcd.print(y_cm);
            lcd.print(")   "); // trailing spaces to clear old digits
            last_x = x_cm;
            last_y = y_cm;
        }

        // Update heading only if changed
        if (theta_deg != last_theta)
        {
            lcd.setCursor(0, 1);
            lcd.print("      ");        // clear old value
            lcd.setCursor(0, 1);
            lcd.print(theta_deg);
            lcd.print((char)223);       // degree symbol
            last_theta = theta_deg;
        }

        vTaskDelayUntil(&xLastWakeTime, displayRate);
    }
}

void vTaskOdometry(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t odomRate = pdMS_TO_TICKS(50); // 50 Hz kaman filtering
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        imu.read();
        odometry_update();

        vTaskDelayUntil(&xLastWakeTime, odomRate);
    }
}
