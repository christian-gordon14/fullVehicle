#include <stdio.h>

#include "motorDriver.h"
#include "wheelSpeeds.h"
#include "imu.h"
#include "bluetooth.h"
#include "scheduler.h"
#include "nvs_flash.h"

void app_main(void)
{
    printf("APP START\n");

    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    printf("NVS OK\n");

    printf("MOTOR INIT\n");
    initializeMotorsPWM();
    printf("MOTOR OK\n");

    printf("WHEEL SPEED INIT\n");
    wheelSpeed_init();
    printf("WHEEL SPEED OK\n");

    printf("IMU INIT\n");
    imu_init();
    printf("IMU OK\n");

    printf("BLUETOOTH INIT\n");
    bluetooth_init();
    printf("BLUETOOTH OK\n");

    printf("SCHEDULER START\n");
    scheduler_start();
    printf("SCHEDULER OK\n");
}

