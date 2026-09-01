#include "bluetooth.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "wheelSpeeds.h"
#include "imu.h"

static const char *TAG = "BLE";

static void start_advertising(void);


/* ============================================================
 * UUIDs
 * ============================================================ */

static const ble_uuid128_t service_uuid =
    BLE_UUID128_INIT(
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34, 0x56, 0x78,
        0x9A, 0xBC, 0xDE, 0xF0
    );

static const ble_uuid128_t characteristic_uuid =
    BLE_UUID128_INIT(
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34, 0x56, 0x78,
        0x9A, 0xBC, 0xDE, 0xF1
    );


/* ============================================================
 * BLE state
 * ============================================================ */

static uint16_t connection_handle =
    BLE_HS_CONN_HANDLE_NONE;

static uint16_t characteristic_handle;

static bool notifications_enabled = false;

/*
 * NimBLE determines the appropriate BLE address type
 * during synchronization.
 */
static uint8_t ble_addr_type;


/* ============================================================
 * GATT characteristic access callback
 * ============================================================ */

static int sensor_data_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    /*
     * We aren't handling READ requests yet.
     * Sensor data will be sent using notifications.
     */

    return 0;
}


/* ============================================================
 * GATT service definition
 * ============================================================ */

static const struct ble_gatt_svc_def gatt_services[] =
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,

        .uuid = &service_uuid.u,

        .characteristics =
        (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &characteristic_uuid.u,

                .access_cb = sensor_data_access,

                .flags =
                    BLE_GATT_CHR_F_READ |
                    BLE_GATT_CHR_F_NOTIFY,

                .val_handle =
                    &characteristic_handle,
            },

            {0}
        }
    },

    {0}
};


/* ============================================================
 * GAP event handler
 * ============================================================ */

static int gap_event_handler(
    struct ble_gap_event *event,
    void *arg
)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:

            if (event->connect.status == 0)
            {
                connection_handle =
                    event->connect.conn_handle;

                ESP_LOGI(
                    TAG,
                    "BLE connected"
                );

                notifications_enabled = false;
            }
            else
            {
                ESP_LOGI(
                    TAG,
                    "BLE connection failed: %d",
                    event->connect.status
                );

                connection_handle =
                    BLE_HS_CONN_HANDLE_NONE;

                notifications_enabled = false;
            }

            break;


        case BLE_GAP_EVENT_DISCONNECT:

            ESP_LOGI(
                TAG,
                "BLE disconnected"
            );

            connection_handle =
                BLE_HS_CONN_HANDLE_NONE;

            notifications_enabled = false;

            /*
             * Start advertising again so another device
             * can connect.
             */
            start_advertising();

            break;


        case BLE_GAP_EVENT_SUBSCRIBE:

            if (event->subscribe.attr_handle ==
                characteristic_handle)
            {
                notifications_enabled =
                    event->subscribe.cur_notify;

                ESP_LOGI(
                    TAG,
                    "Notifications: %s",
                    notifications_enabled ?
                    "enabled" :
                    "disabled"
                );
            }

            break;


        case BLE_GAP_EVENT_ADV_COMPLETE:

            ESP_LOGI(
                TAG,
                "Advertising complete"
            );

            start_advertising();

            break;


        default:
            break;
    }

    return 0;
}


/* ============================================================
 * Start advertising
 * ============================================================ */

static void start_advertising(void)
{
    int rc;

    struct ble_hs_adv_fields fields;

    memset(
        &fields,
        0,
        sizeof(fields)
    );

    /*
     * General discoverable + BLE only.
     */

    fields.flags =
        BLE_HS_ADV_F_DISC_GEN |
        BLE_HS_ADV_F_BREDR_UNSUP;


    /*
     * Device name.
     */

    const char *name =
        "ESP32_VEHICLE";

    fields.name =
        (uint8_t *)name;

    fields.name_len =
        strlen(name);

    fields.name_is_complete = 1;


    /*
     * Configure advertising data.
     */

    rc = ble_gap_adv_set_fields(
        &fields
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed to set advertising fields: %d",
            rc
        );

        return;
    }


    /*
     * Configure advertising parameters.
     */

    struct ble_gap_adv_params adv_params;

    memset(
        &adv_params,
        0,
        sizeof(adv_params)
    );

    adv_params.conn_mode =
        BLE_GAP_CONN_MODE_UND;

    adv_params.disc_mode =
        BLE_GAP_DISC_MODE_GEN;


    /*
     * Start advertising.
     *
     * Use the address type determined by
     * ble_hs_id_infer_auto().
     */

    rc = ble_gap_adv_start(
        ble_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event_handler,
        NULL
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed to start advertising: %d",
            rc
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "BLE advertising started"
    );
}


/* ============================================================
 * BLE synchronization callback
 * ============================================================ */

static void ble_on_sync(void)
{
    int rc;

    /*
     * Make sure the ESP32 has a valid BLE address.
     *
     * IMPORTANT:
     * The second argument must be a valid pointer.
     * Passing NULL here caused the StoreProhibited crash.
     */

    rc = ble_hs_id_infer_auto(
        0,
        &ble_addr_type
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed to infer BLE address: %d",
            rc
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "BLE address type: %d",
        ble_addr_type
    );

    start_advertising();
}


/* ============================================================
 * NimBLE host task
 * ============================================================ */

static void ble_host_task(void *param)
{
    /*
     * Run the NimBLE host.
     */

    nimble_port_run();

    /*
     * This returns when NimBLE shuts down.
     */

    nimble_port_freertos_deinit();
}


/* ============================================================
 * Initialization
 * ============================================================ */

void bluetooth_init(void)
{
    esp_err_t ret;


    /*
     * Initialize NVS.
     */

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    /*
     * Initialize NimBLE.
     */

    ret = nimble_port_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "NimBLE initialization failed: %d",
            ret
        );

        return;
    }


    /*
     * Initialize standard GAP and GATT services.
     */

    ble_svc_gap_init();

    ble_svc_gatt_init();


    /*
     * Set device name.
     */

    ret = ble_svc_gap_device_name_set(
        "ESP32_VEHICLE"
    );

    if (ret != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed to set BLE device name: %d",
            ret
        );

        return;
    }


    /*
     * Register our custom GATT service.
     */

    int rc;

    rc = ble_gatts_count_cfg(
        gatt_services
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "ble_gatts_count_cfg failed: %d",
            rc
        );

        return;
    }


    rc = ble_gatts_add_svcs(
        gatt_services
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "ble_gatts_add_svcs failed: %d",
            rc
        );

        return;
    }


    /*
     * Configure NimBLE callbacks.
     */

    ble_hs_cfg.sync_cb =
        ble_on_sync;


    /*
     * Start NimBLE host task.
     */

    nimble_port_freertos_init(
        ble_host_task
    );

    ESP_LOGI(
        TAG,
        "Bluetooth initialized"
    );
}


/* ============================================================
 * Send sensor data
 * ============================================================ */

void bluetooth_send_sensor_data(void)
{
    /*
     * Make sure we are connected.
     */

    if (connection_handle ==
        BLE_HS_CONN_HANDLE_NONE)
    {
        return;
    }


    /*
     * Make sure the client subscribed to
     * notifications.
     */

    if (!notifications_enabled)
    {
        return;
    }


    /*
     * Get wheel speeds.
     */

    float fl =
        wheelSpeed_get(WHEEL_FL);

    float fr =
        wheelSpeed_get(WHEEL_FR);

    float rl =
        wheelSpeed_get(WHEEL_RL);

    float rr =
        wheelSpeed_get(WHEEL_RR);


    /*
     * Get IMU data.
     */

    AccelValues accel =
        imu_get_accel();

    GyroValues gyro =
        imu_get_gyro();


    /*
     * Format data as CSV.
     *
     * FL,FR,RL,RR,ax,ay,az,gx,gy,gz
     */

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


    /*
     * Convert data into a NimBLE mbuf.
     */

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


    /*
     * Send notification.
     */

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
            "BLE notification failed: %d",
            rc
        );
    }
}