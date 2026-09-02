#include <stdio.h>

#include "motorDriver.h"
#include "wheelSpeeds.h"
#include "imu.h"
#include "bluetooth.h"
#include "scheduler.h"
#include "nvs_flash.h"

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();

    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_result);

    initializeMotorsPWM();
    wheelSpeed_init();
    imu_init();
    bluetooth_init();

    scheduler_start();
}

