#include "wheelSpeeds.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"




// ============================================================
// Configuration
// ============================================================

// ADC2
#define HALL_CHANNEL_FL    ADC_CHANNEL_3
#define HALL_CHANNEL_RL    ADC_CHANNEL_6

// ADC1
#define HALL_CHANNEL_FR    ADC_CHANNEL_0
#define HALL_CHANNEL_RR    ADC_CHANNEL_3

// ============================================================
// Private variables
// ============================================================

static const char *TAG = "WHEEL_SPEED";

// ADC handles
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

// ADC calibration handles
static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_cali_handle_t adc2_cali_handle = NULL;


// Wheel state
// this is creating 4 instances that are empty
static WheelSpeed wheel_speeds[WHEEL_COUNT] = {0};

static const HallADCConfig hall_adc_config[WHEEL_COUNT] =
{
    // setting the units and channel for each sensor/instance
    [WHEEL_FL] =
    {
        .unit = ADC_UNIT_2,
        .channel = HALL_CHANNEL_FL
    },

    [WHEEL_FR] =
    {
        .unit = ADC_UNIT_1,
        .channel = HALL_CHANNEL_FR
    },

    [WHEEL_RL] =
    {
        .unit = ADC_UNIT_2,
        .channel = HALL_CHANNEL_RL
    },

    [WHEEL_RR] =
    {
        .unit = ADC_UNIT_1,
        .channel = HALL_CHANNEL_RR
    }
};


// ============================================================
// Private functions
// ============================================================

static void initializeADC(void);
static void read_hall_sensors(void);
static void updateWheelSpeed(Wheel wheel);
static void lowPassFilter(float *current_value, float *filtered_value, float LPF_ALPHA);


static void lowPassFilter(float *current_value, float *filtered_value, float LPF_ALPHA)
{
    *filtered_value = LPF_ALPHA * (*current_value) + (1.0f - LPF_ALPHA) * (*filtered_value);
}

static void initializeADC(void)
{
    adc_oneshot_unit_init_cfg_t adc1_init =
    {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_init, &adc1_handle));

    adc_oneshot_unit_init_cfg_t adc2_init =
    {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_init, &adc2_handle));

    adc_oneshot_chan_cfg_t adc_config =
    {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle,HALL_CHANNEL_FR,&adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle,HALL_CHANNEL_RR,&adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle,HALL_CHANNEL_FL,&adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle,HALL_CHANNEL_RL,&adc_config));
    ESP_LOGI(TAG, "ADC initialized");
}

static void read_hall_sensors(void)
{
    for (Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        // creating a pointer for each wheel names config
        const HallADCConfig *config = &hall_adc_config[wheel];
        
        //creating a pointer for a hall value
        int *hall_value = &wheel_speeds[wheel].hall_value_raw;

        if (config->unit == ADC_UNIT_1)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, config->channel, hall_value));
        }
        else
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, config->channel, hall_value));
        }
    }
}

// this will go over 1 'wheel' item at a time based on the function calling
static void updateWheelSpeed(Wheel wheel)
{
    // creating a pointer for each instance (one at a time)
    WheelSpeed *ws = &wheel_speeds[wheel];
    float wheel_speed_timeout_us;
    int64_t this_time = esp_timer_get_time();
    ws->new_measurement = false;

    // setting the timeout based on wheel speed
    if (ws->wheel_speed_measured > 0.0f)
    {
        wheel_speed_timeout_us = MAX_MISSED_PULSES * (2.0f * PI * 1.0e6f) / (TOTAL_MAGNETS * ws->wheel_speed_measured);
    }
    else
    {
        wheel_speed_timeout_us = 1.0e6f;
    }

    // finding the dt between pulses and actually calculating wheel speed
    if ((ws->hall_value_raw >= HALL_THRESHOLD) && (ws->previous_hall_value_raw < HALL_THRESHOLD))
    {
        if (ws->last_time == 0)
        {
            ws->last_time = this_time;
        }

        else
        {
            int64_t dt = this_time - ws->last_time;
            if (dt > 0 && dt <= wheel_speed_timeout_us)
            {
                ws->wheel_speed_measured = (2.0f * PI / TOTAL_MAGNETS) / ((float)dt / 1.0e6f);
                ws->new_measurement = true;
            }
            ws->last_time = this_time;
        }
    }

    // settign the wheel speed to zero
    if (ws->last_time != 0 && (this_time - ws->last_time) > wheel_speed_timeout_us)
    {
        ws->wheel_speed_measured = 0.0f;
        ws->new_measurement = true;
    }

    // filtering the wheel speed
    lowPassFilter(&ws->wheel_speed_measured, &ws->wheel_speed_filtered, LPF_ALPHA_WHEEL_SPEEDS);
    ws->previous_hall_value_raw = ws->hall_value_raw;
}

// ============================================================
// Public functions
// ============================================================

void wheelSpeed_init(void)
{
    initializeADC();
    ESP_LOGI(TAG, "Wheel speed system initialized");
}

void wheelSpeed_update(void)
{
    read_hall_sensors();
    for (Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        updateWheelSpeed(wheel);
    }
}

float wheelSpeed_get(Wheel wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return 0.0f;
    }
    return wheel_speeds[wheel].wheel_speed_filtered;
}

bool wheelSpeed_newMeasurement(Wheel wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return false;
    }
    return wheel_speeds[wheel].new_measurement;
}
