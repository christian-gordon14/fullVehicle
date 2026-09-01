#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>

void bluetooth_init(void);
void bluetooth_stop(void);

bool bluetooth_notify_sensor(const char *data);

bool writeSpeed(void);

#endif