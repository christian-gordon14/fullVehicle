#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "wheelSpeeds.h"

#define PWM_MAX_DUTY 1023
#define MAX_VOLTAGE 7.f
#define MIN_VOLTAGE 0.f

typedef struct{
    gpio_num_t in;
    gpio_num_t out;
    gpio_num_t pwm;
    ledc_channel_t pwm_channel;
} MotorPins;

void initializeMotorsPWM(void);
void motorDriver_setVoltage(Wheel wheel, float voltage);

#endif