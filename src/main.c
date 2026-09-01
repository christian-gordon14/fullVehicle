#include <stdio.h>

#include "motorDriver.h"
#include "wheelSpeeds.h"
#include "imu.h"
#include "bluetooth.h"
#include "scheduler.h"

void app_main(void)
{
    // Initialize hardware
    initializeMotorsPWM();
    wheelSpeed_init();
    imu_init();
    bluetooth_init();

    // Main control loop
    scheduler_start();
}

