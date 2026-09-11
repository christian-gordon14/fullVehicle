#ifndef WHEEL_SPEEDS_H
#define WHEEL_SPEEDS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#define PI                  3.14159f
#define HALL_THRESHOLD      1900
#define TOTAL_MAGNETS       8
#define MAX_MISSED_PULSES   1
#define LPF_ALPHA_WHEEL_SPEEDS 0.1f

typedef enum
{
    WHEEL_FL = 0,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
    WHEEL_COUNT
} Wheel;

typedef struct
{
    int hall_value_raw;
    int previous_hall_value_raw;
    int64_t last_time;
    float wheel_speed_measured;
    float wheel_speed_filtered;

    bool new_measurement;

} WheelSpeed;

typedef struct
{
    adc_unit_t unit;
    adc_channel_t channel;
} HallADCConfig;

void wheelSpeed_init(void);
void wheelSpeed_update(void);
float wheelSpeed_get(Wheel wheel);
bool wheelSpeed_newMeasurement(Wheel wheel);

#endif