#include "motorDriver.h"
#include "wheelSpeeds.h"

#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

// ============================================================
// Configuration
// ============================================================

// H-bridge 1 (left wheels)
// Front Motor
#define ENA_PIN_LEFT 2
#define IN1_PIN_LEFT 16
#define IN2_PIN_LEFT 4 // these are reversed on the car

// Rear Motor
#define ENB_PIN_LEFT 19
#define IN3_PIN_LEFT 23
#define IN4_PIN_LEFT 17 // these are reversed on the car

// H-bridge 2 (right wheels)
// Front Motor
#define ENA_PIN_RIGHT 12
#define IN1_PIN_RIGHT 32
#define IN2_PIN_RIGHT 33

// Rear Motor
#define ENB_PIN_RIGHT 27
#define IN3_PIN_RIGHT 25
#define IN4_PIN_RIGHT 26

// ============================================================
// Function definitions
// ============================================================
static void initializeMotor(Wheel wheel);
static void initializePWM(Wheel wheel);
static void initializePWMTimer(void);
void initializeMotorsPWM(void);

// ============================================================
// Private variables
// ============================================================
static MotorPins motorPins[WHEEL_COUNT] = {
    [WHEEL_FL] = {
        .in = IN1_PIN_LEFT,
        .out = IN2_PIN_LEFT,
        .pwm = ENA_PIN_LEFT,
        .pwm_channel = LEDC_CHANNEL_0
    },

    [WHEEL_FR] = {
        .in = IN1_PIN_RIGHT,
        .out = IN2_PIN_RIGHT,
        .pwm = ENA_PIN_RIGHT,
        .pwm_channel = LEDC_CHANNEL_1
    },

    [WHEEL_RL] = {
        .in = IN3_PIN_LEFT,
        .out = IN4_PIN_LEFT,
        .pwm = ENB_PIN_LEFT,
        .pwm_channel = LEDC_CHANNEL_2
    },

    [WHEEL_RR] = {
        .in = IN3_PIN_RIGHT,
        .out = IN4_PIN_RIGHT,
        .pwm = ENB_PIN_RIGHT,
        .pwm_channel = LEDC_CHANNEL_3
    }
};


// ============================================================
// Private functions
// ============================================================
static void initializeMotor(Wheel motor)
{
    const MotorPins *pins = &motorPins[motor];
    gpio_set_direction(pins->in, GPIO_MODE_OUTPUT);
    gpio_set_direction(pins->out, GPIO_MODE_OUTPUT);
    // Forward direction
    gpio_set_level(pins->in, 1);
    gpio_set_level(pins->out, 0);
}

static void initializePWMTimer(void)
{
    ledc_timer_config_t pwm_timer =
    {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
}

static void initializePWM(Wheel motor)
{
    const MotorPins *pins = &motorPins[motor];
    ledc_channel_config_t pwm_channel = {
    .gpio_num = pins->pwm,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = pins->pwm_channel,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
}

// ============================================================
// Public functions
// ============================================================
void initializeMotorsPWM(void)
{
    initializePWMTimer();
    for(int wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        initializeMotor(wheel);
        initializePWM(wheel);
    }
}

void motorDriver_setVoltage(Wheel wheel, float voltage)
{
    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    // Limit voltage
    if (voltage > MAX_VOLTAGE)
    {
        voltage = MAX_VOLTAGE;
    }
    else if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }

    // Convert voltage to PWM duty
    uint32_t duty =
        (uint32_t)((voltage / MAX_VOLTAGE) * PWM_MAX_DUTY);

    const MotorPins *pins = &motorPins[wheel];

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            pins->pwm_channel,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            pins->pwm_channel
        )
    );
}