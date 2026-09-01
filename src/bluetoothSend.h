#ifndef BLUETOOTH_SEND_H
#define BLUETOOTH_SEND_H

#include <stdbool.h>
#include <stdint.h>

void bluetooth_send_init(void);
void bluetooth_send_sensor_data(void);

void bluetooth_send_set_connection(
    uint16_t conn_handle
);

void bluetooth_send_clear_connection(void);

void bluetooth_send_set_notification(
    bool enabled
);

void bluetooth_send_set_characteristic_handle(
    uint16_t handle
);

#endif