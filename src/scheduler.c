#include "scheduler.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motorDriver.h"
#include "wheelSpeeds.h"
#include "imu.h"
#include "controller.h"
#include "bluetoothSend.h"
#include "bluetoothReceive.h"
#include "sharedFunctions.h"
// ============================================================
// Configuration
// ============================================================

// ============================================================
// Function definitions
// ============================================================
static void imuTask(void *pvParameters);
static void wheelSpeedSensorsTask(void *pvParameters);
static void controllerTask(void *pvParameters);
static void blueToothTask(void *pvParameters);
// ============================================================
// Private variables
// ============================================================

// ============================================================
// Private functions
// ============================================================
static void imuTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        imu_update();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1));
    }
}

static void wheelSpeedSensorsTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        wheelSpeed_update();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

static void controllerTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        updateControl();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

static void blueToothTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        bluetooth_send_sensor_data();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
    }
}

static void vehicleVelocityEstimationTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        kalmanFilter();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1));
    }
}
// ============================================================
// Public functions
// ============================================================

void scheduler_start(void)
{
    xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(wheelSpeedSensorsTask, "Wheel Speed", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(vehicleVelocityEstimationTask, "Velocity Estimate", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(controllerTask, "Controller", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(blueToothTask, "Bluetooth", 4096, NULL, 2, NULL, 0);
    printf("Tasks created\n");
}