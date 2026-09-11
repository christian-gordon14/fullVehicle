#include "bluetoothSend.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "host/ble_hs.h"
#include "host/ble_gatt.h"

#include "wheelSpeeds.h"
#include "imu.h"
#include "controller.h"
#include "vehicleDynamics.h"

static uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t characteristic_handle = 0;
static bool notifications_enabled = false;

void bluetooth_send_init(void)
{
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    characteristic_handle = 0;
    notifications_enabled = false;
}

void bluetooth_send_set_connection(uint16_t conn_handle)
{
    connection_handle = conn_handle;
}

void bluetooth_send_clear_connection(void)
{
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    notifications_enabled = false;
}

void bluetooth_send_set_notification(bool enabled)
{
    notifications_enabled = enabled;
}

void bluetooth_send_set_characteristic_handle(uint16_t handle)
{
    characteristic_handle = handle;
}

void bluetooth_send_sensor_data(void)
{
    if (connection_handle == BLE_HS_CONN_HANDLE_NONE)
        return;

    if (!notifications_enabled)
        return;

    if (characteristic_handle == 0)
        return;

    float fl = wheelSpeed_get(WHEEL_FL);
    float fr = wheelSpeed_get(WHEEL_FR);
    float rl = wheelSpeed_get(WHEEL_RL);
    float rr = wheelSpeed_get(WHEEL_RR);

    AccelValues accel = imu_get_accel();
    GyroValues gyro = imu_get_gyro();
    VehicleStates vehicleStates = imu_get_states();

    Controller_outputs controller_outputs = updateControl();

    WheelStruct wheelForces = getWheelForces();
    WheelStruct slipRatios = getSlipRatios();
    float ax_gravity = imu_get_accel_terms();

    char buffer[256];

    int length = snprintf(
        buffer,
        sizeof(buffer),
        "%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f, %.4f, %.4f,"
        "%.4f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%.4f",
        // "%.4f,%.4f,%.4f,%.4f,"
        // "%.4f,%.4f,%.4f,%.4f",


        fl, fr, rl, rr,

        accel.ax,
        accel.ay,
        accel.az,

        gyro.gx,
        gyro.gy,
        gyro.gz,

        vehicleStates.heading,
        vehicleStates.xVelocity,
        vehicleStates.pitch,

        controller_outputs.feedforward,

        controller_outputs.PID_wheel_speed_FL,
        controller_outputs.PID_wheel_speed_FR,
        controller_outputs.PID_wheel_speed_RL,
        controller_outputs.PID_wheel_speed_RR,
        ax_gravity

        // controller_outputs.PID_heading,

        // wheelForces.WHEEL_FL,
        // wheelForces.WHEEL_FR,
        // wheelForces.WHEEL_RL,
        // wheelForces.WHEEL_RR,

        // slipRatios.WHEEL_FL,
        // slipRatios.WHEEL_FR,
        // slipRatios.WHEEL_RL,
        // slipRatios.WHEEL_RR
    );

    if (length <= 0)
        return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer, length);

    if (om == NULL)
        return;

    ble_gatts_notify_custom(
        connection_handle,
        characteristic_handle,
        om
    );
}