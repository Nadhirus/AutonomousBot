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
void vTaskDiffDriveUpdate(void *pvParameters);
void vTaskDummyDrive(void *pvParameters);
void vTaskDisplayOdometry(void *pvParameters);
void vTaskIMUSerial(void *pvParameters);

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

    xTaskCreate(vTaskDiffDriveUpdate, "DiffUpdate", 192, NULL, 2, NULL); //192
    xTaskCreate(vTaskDummyDrive,       "DummyDrive", 128, NULL, 1, NULL);
    xTaskCreate(vTaskDisplayOdometry,  "DisplayOdo", 128, NULL, 1, NULL); //192
    xTaskCreate(vTaskIMUSerial,        "IMUSerial", 384, NULL, 3, NULL);

    vTaskStartScheduler();
}

void loop()
{
}

void vTaskDiffDriveUpdate(void *pvParameters)
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

void vTaskDummyDrive(void *pvParameters)
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

void vTaskDisplayOdometry(void *pvParameters)
{
    (void)pvParameters;
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

void vTaskIMUSerial(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        imu.read();

        snprintf_P(report, sizeof(report),
                   PSTR("A: %6d %6d %6d    M: %6d %6d %6d    G: %6d %6d %6d"),
                   imu.a.x, imu.a.y, imu.a.z,
                   imu.m.x, imu.m.y, imu.m.z,
                   imu.g.x, imu.g.y, imu.g.z);

        Serial.println(report);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}
