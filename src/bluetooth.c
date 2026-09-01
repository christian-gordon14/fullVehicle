#include "bluetooth.h"
#include "bluetoothReceive.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE";

#define DEVICE_NAME "ESP32_VEHICLE"

/* ============================================================
 * UUIDs
 *
 * Python sees these UUIDs:
 *
 * SENSOR:
 * f1debc9a-7856-3412-7856-341278563412
 *
 * COMMAND:
 * f2debc9a-7856-3412-7856-341278563412
 *
 * SERVICE:
 * 12345678-1234-5678-1234-56789abcdef0
 *
 * IMPORTANT:
 * BLE_UUID128_INIT() uses the little-endian byte representation
 * used internally by NimBLE.
 * ============================================================ */

/* Python -> ESP32 */
/* ESP32 -> Python */
static const ble_uuid128_t sensor_uuid =
    BLE_UUID128_INIT(
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34,
        0x56, 0x78,
        0x12, 0x34,
        0x56, 0x78,
        0x9a, 0xbc, 0xde, 0xf1
    );

static const ble_uuid128_t command_uuid =
    BLE_UUID128_INIT(
        0x12, 0x34, 0x56, 0x78,
        0x12, 0x34,
        0x56, 0x78,
        0x12, 0x34,
        0x56, 0x78,
        0x9a, 0xbc, 0xde, 0xf2
    );

static const ble_uuid128_t service_uuid =
    BLE_UUID128_INIT(
        0xf0, 0xde, 0xbc, 0x9a,
        0x78, 0x56,
        0x34, 0x12,
        0x78, 0x56,
        0x34, 0x12,
        0x78, 0x56, 0x34, 0x12
    );

/* ============================================================
 * BLE state
 * ============================================================ */

static uint8_t ble_addr_type;

static uint16_t sensor_val_handle = 0;
static uint16_t command_val_handle = 0;

static uint16_t current_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* ============================================================
 * Forward declarations
 * ============================================================ */

static void ble_on_sync(void);
static void ble_on_reset(int reason);
static void ble_host_task(void *param);

static int ble_gap_event(
    struct ble_gap_event *event,
    void *arg
);

static int sensor_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
);

static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
);

/* ============================================================
 * Sensor characteristic
 * ============================================================ */

static int sensor_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    if (ctxt == NULL)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        /*
         * Nothing is returned for a normal READ.
         *
         * Sensor data is sent using notifications.
         */
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ============================================================
 * Command characteristic
 * ============================================================ */

static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    if (ctxt == NULL)
    {
        ESP_LOGE(TAG, "NULL GATT context");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        ESP_LOGW(TAG, "Unexpected GATT operation");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->om == NULL)
    {
        ESP_LOGE(TAG, "NULL mbuf");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t length = OS_MBUF_PKTLEN(ctxt->om);

    ESP_LOGI(
        TAG,
        "Received command: %u bytes",
        length
    );

    if (length == 0)
    {
        ESP_LOGW(TAG, "Empty command");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    char command[32];

    if (length >= sizeof(command))
    {
        length = sizeof(command) - 1;
    }

    int rc = ble_hs_mbuf_to_flat(
        ctxt->om,
        command,
        length,
        NULL
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "ble_hs_mbuf_to_flat failed: %d",
            rc
        );

        return BLE_ATT_ERR_UNLIKELY;
    }

    command[length] = '\0';

    ESP_LOGI(
        TAG,
        "Command received: '%s'",
        command
    );

    /*
     * Pass the original GATT context to bluetoothReceive.c.
     */
    return bluetooth_receive_access(
        conn_handle,
        attr_handle,
        ctxt,
        arg
    );
}

/* ============================================================
 * GATT services
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
                .uuid = &sensor_uuid.u,

                .access_cb = sensor_access,

                .val_handle = &sensor_val_handle,

                .flags =
                    BLE_GATT_CHR_F_READ |
                    BLE_GATT_CHR_F_NOTIFY,
            },

            {
                .uuid = &command_uuid.u,

                .access_cb = command_access,

                .val_handle = &command_val_handle,

                .flags =
                    BLE_GATT_CHR_F_WRITE |
                    BLE_GATT_CHR_F_WRITE_NO_RSP,
            },

            {
                0
            }
        }
    },

    {
        0
    }
};

/* ============================================================
 * GAP event handler
 * ============================================================ */

static int ble_gap_event(
    struct ble_gap_event *event,
    void *arg
)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:

            if (event->connect.status == 0)
            {
                current_conn_handle =
                    event->connect.conn_handle;

                ESP_LOGI(
                    TAG,
                    "BLE connected, conn_handle=%u",
                    current_conn_handle
                );
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "BLE connection failed: %d",
                    event->connect.status
                );

                current_conn_handle =
                    BLE_HS_CONN_HANDLE_NONE;

                ble_on_sync();
            }

            break;

        case BLE_GAP_EVENT_DISCONNECT:

            ESP_LOGI(
                TAG,
                "BLE disconnected, reason=%d",
                event->disconnect.reason
            );

            current_conn_handle =
                BLE_HS_CONN_HANDLE_NONE;

            bluetooth_receive_stop();

            ble_on_sync();

            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:

            ESP_LOGI(
                TAG,
                "BLE advertising complete"
            );

            ble_on_sync();

            break;

        case BLE_GAP_EVENT_NOTIFY_TX:

            ESP_LOGD(
                TAG,
                "Notification TX: conn=%u attr=%u status=%d",
                event->notify_tx.conn_handle,
                event->notify_tx.attr_handle,
                event->notify_tx.status
            );

            break;

        default:
            break;
    }

    return 0;
}

/* ============================================================
 * BLE advertising
 * ============================================================ */

static void ble_on_sync(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    struct ble_gap_adv_params adv_params;

    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    /* Primary advertisement: flags + service UUID only */
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.uuids128 = (ble_uuid128_t *)&service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    /* Scan response: device name */
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    rsp_fields.name = (uint8_t *)DEVICE_NAME;
    rsp_fields.name_len = strlen(DEVICE_NAME);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                            &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising as %s", DEVICE_NAME);
}

/* ============================================================
 * BLE reset callback
 * ============================================================ */

static void ble_on_reset(int reason)
{
    ESP_LOGE(
        TAG,
        "BLE reset, reason=%d",
        reason
    );

    current_conn_handle =
        BLE_HS_CONN_HANDLE_NONE;
}

/* ============================================================
 * BLE host task
 * ============================================================ */

static void ble_host_task(void *param)
{
    ESP_LOGI(
        TAG,
        "BLE host task started"
    );

    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* ============================================================
 * Public initialization
 * ============================================================ */

void bluetooth_init(void)
{
    ESP_LOGI(
        TAG,
        "Initializing BLE"
    );

    int rc = nimble_port_init();

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "nimble_port_init failed: %d",
            rc
        );

        return;
    }

    ble_hs_cfg.reset_cb =
        ble_on_reset;

    ble_hs_cfg.sync_cb =
        ble_on_sync;

    ble_svc_gap_init();

    ble_svc_gatt_init();

    rc = ble_svc_gap_device_name_set(
        DEVICE_NAME
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "ble_svc_gap_device_name_set failed: %d",
            rc
        );

        return;
    }

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

    ESP_LOGI(
        TAG,
        "GATT services registered"
    );

    ESP_LOGI(
        TAG,
        "Sensor characteristic handle: %u",
        sensor_val_handle
    );

    ESP_LOGI(
        TAG,
        "Command characteristic handle: %u",
        command_val_handle
    );

    nimble_port_freertos_init(
        ble_host_task
    );
}

/* ============================================================
 * Public stop
 * ============================================================ */

void bluetooth_stop(void)
{
    bluetooth_receive_stop();

    ESP_LOGI(
        TAG,
        "Bluetooth stop requested"
    );
}

/* ============================================================
 * Send sensor data to Python
 *
 * Example:
 *
 * bluetooth_notify_sensor(
 *     "1.0000,2.0000,3.0000,..."
 * );
 *
 * This sends the string through the SENSOR characteristic.
 * ============================================================ */

bool bluetooth_notify_sensor(
    const char *data
)
{
    if (data == NULL)
    {
        return false;
    }

    if (current_conn_handle ==
        BLE_HS_CONN_HANDLE_NONE)
    {
        return false;
    }

    if (sensor_val_handle == 0)
    {
        return false;
    }

    uint16_t length =
        strlen(data);

    if (length == 0)
    {
        return false;
    }

    int rc = ble_gatts_notify_custom(
        current_conn_handle,
        sensor_val_handle,
        NULL
    );

    if (rc != 0)
    {
        ESP_LOGE(
            TAG,
            "ble_gatts_notify_custom failed: %d",
            rc
        );

        return false;
    }

    return true;
}

/* ============================================================
 * Existing interface
 * ============================================================ */

bool writeSpeed(void)
{
    return bluetooth_get_move_command();
}