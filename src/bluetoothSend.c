#include "bluetoothSend.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"

#include "host/ble_hs.h"
#include "host/ble_gatt.h"

#include "wheelSpeeds.h"
#include "imu.h"

static const char *TAG = "BLE_TX";

static uint16_t connection_handle =
    BLE_HS_CONN_HANDLE_NONE;

static uint16_t characteristic_handle = 0;

static bool notifications_enabled = false;

void bluetooth_send_init(void)
{
    connection_handle =
        BLE_HS_CONN_HANDLE_NONE;

    characteristic_handle = 0;

    notifications_enabled = false;
}

void bluetooth_send_set_connection(
    uint16_t conn_handle)
{
    connection_handle = conn_handle;
}

void bluetooth_send_clear_connection(void)
{
    connection_handle =
        BLE_HS_CONN_HANDLE_NONE;

    notifications_enabled = false;
}

void bluetooth_send_set_notification(
    bool enabled)
{
    notifications_enabled = enabled;
}

void bluetooth_send_set_characteristic_handle(
    uint16_t handle)
{
    characteristic_handle = handle;
}

void bluetooth_send_sensor_data(void)
{
    if (connection_handle ==
        BLE_HS_CONN_HANDLE_NONE)
    {
        return;
    }

    if (!notifications_enabled)
    {
        return;
    }

    if (characteristic_handle == 0)
    {
        return;
    }

    float fl =
        wheelSpeed_get(WHEEL_FL);

    float fr =
        wheelSpeed_get(WHEEL_FR);

    float rl =
        wheelSpeed_get(WHEEL_RL);

    float rr =
        wheelSpeed_get(WHEEL_RR);

    AccelValues accel =
        imu_get_accel();

    GyroValues gyro =
        imu_get_gyro();

    char buffer[256];

    int length = snprintf(
        buffer,
        sizeof(buffer),
        "%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f",
        fl,
        fr,
        rl,
        rr,
        accel.ax,
        accel.ay,
        accel.az,
        gyro.gx,
        gyro.gy,
        gyro.gz
    );

    if (length <= 0)
    {
        return;
    }

    struct os_mbuf *om =
        ble_hs_mbuf_from_flat(
            buffer,
            length
        );

    if (om == NULL)
    {
        ESP_LOGW(
            TAG,
            "Failed to allocate BLE mbuf"
        );

        return;
    }

    int rc =
        ble_gatts_notify_custom(
            connection_handle,
            characteristic_handle,
            om
        );

    if (rc != 0)
    {
        ESP_LOGW(
            TAG,
            "Notification failed: %d",
            rc
        );
    }
}