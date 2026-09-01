#ifndef BLUETOOTH_RECEIVE_H
#define BLUETOOTH_RECEIVE_H

#include <stdbool.h>
#include <stdint.h>
#include "host/ble_gatt.h"

void bluetooth_receive_init(void);
void bluetooth_receive_stop(void);
bool bluetooth_get_move_command(void);

int bluetooth_receive_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
);

#endif