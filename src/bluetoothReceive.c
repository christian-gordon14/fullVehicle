#include "bluetoothReceive.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_gatt.h"
#include "os/os_mbuf.h"

static const char *TAG = "BLE_RX";

static volatile bool vehicle_move_command = false;

void bluetooth_receive_init(void)
{
    vehicle_move_command = false;
    ESP_LOGI(TAG, "Receive initialized: MOVE = OFF");
}

void bluetooth_receive_stop(void)
{
    vehicle_move_command = false;
    ESP_LOGI(TAG, "Receive stopped: MOVE = OFF");
}

bool bluetooth_get_move_command(void)
{
    return vehicle_move_command;
}

int bluetooth_receive_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "BLE WRITE CALLBACK");
    ESP_LOGI(TAG, "================================");

    if (ctxt == NULL)
    {
        ESP_LOGE(TAG, "NULL GATT context");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        ESP_LOGW(TAG, "Not a characteristic write: op=%d", ctxt->op);
        return 0;
    }

    if (ctxt->om == NULL)
    {
        ESP_LOGE(TAG, "NULL mbuf");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t length = OS_MBUF_PKTLEN(ctxt->om);

    if (length == 0)
    {
        ESP_LOGW(TAG, "Empty BLE command");
        return 0;
    }

    char command[32];

    if (length >= sizeof(command))
    {
        length = sizeof(command) - 1;
    }

    int rc = os_mbuf_copydata(
        ctxt->om,
        0,
        length,
        command
    );

    if (rc != 0)
    {
        ESP_LOGE(TAG, "os_mbuf_copydata failed: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }

    command[length] = '\0';

    ESP_LOGI(TAG, "Received command: '%s'", command);

    /*
     * Remove CR/LF if the computer sends them.
     */
    for (int i = 0; command[i] != '\0'; i++)
    {
        if (command[i] == '\r' || command[i] == '\n')
        {
            command[i] = '\0';
            break;
        }
    }

    if (strcmp(command, "UP") == 0)
    {
        vehicle_move_command = true;

        ESP_LOGI(TAG, "************ UP ************");
        ESP_LOGI(TAG, "vehicle_move_command = %d",
                 vehicle_move_command);
    }
    else if (strcmp(command, "STOP") == 0)
    {
        vehicle_move_command = false;

        ESP_LOGI(TAG, "*********** STOP ***********");
        ESP_LOGI(TAG, "vehicle_move_command = %d",
                 vehicle_move_command);
    }
    else
    {
        ESP_LOGW(TAG, "Unknown command: '%s'", command);
    }

    return 0;
}