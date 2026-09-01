#include "controller.h"
#include "wheelSpeeds.h"
#include "motorDriver.h"
#include <stdio.h>

// ============================================================
// Configuration
// ============================================================
#define DT 0.002
#define RESISTANCE_ohms 6.4f
#define DAMPING 0.001f
#define KM 0.275f
#define KN 0.2540f

#define KP_WHEEL_SPEED 0.8f
#define KI_WHEEL_SPEED 1.28f
#define KD_WHEEL_SPEED 0.0f
// ============================================================
// Private variables
// ============================================================
static PID_params wheel_speed_pid[WHEEL_COUNT] = {
    [WHEEL_FL] = {
    .Kp = KP_WHEEL_SPEED,
    .Ki = KI_WHEEL_SPEED,
    .Kd = KD_WHEEL_SPEED,
    .max = MAX_VOLTAGE,
    .min = MIN_VOLTAGE,
    },

    [WHEEL_FR] = {
    .Kp = KP_WHEEL_SPEED,
    .Ki = KI_WHEEL_SPEED,
    .Kd = KD_WHEEL_SPEED,
    .max = MAX_VOLTAGE,
    .min = MIN_VOLTAGE,
    },

    [WHEEL_RL] = {
    .Kp = KP_WHEEL_SPEED,
    .Ki = KI_WHEEL_SPEED,
    .Kd = KD_WHEEL_SPEED,
    .max = MAX_VOLTAGE,
    .min = MIN_VOLTAGE,
    },

    [WHEEL_RR] = {
    .Kp = KP_WHEEL_SPEED,
    .Ki = KI_WHEEL_SPEED,
    .Kd = KD_WHEEL_SPEED,
    .max = MAX_VOLTAGE,
    .min = MIN_VOLTAGE,
    },
};

// ============================================================
// Public variables
// ============================================================

float wheel_speed_targets[WHEEL_COUNT] = {
    [WHEEL_FL] = 13.f,
    [WHEEL_FR] = 13.f,
    [WHEEL_RL] = 13.f,
    [WHEEL_RR] = 13.f,
};

// ============================================================
// Private functions
// ============================================================

// ============================================================
// Public functions
// ============================================================
float PIDController(PID_params *controller, float measurement){
    PID_params *ct = controller;
    float last_error = ct->error;
    ct->error = ct->target - measurement;
    ct->integral_candidate = ct->integral + DT * ct->error;
    float error_deriv = (ct->error - last_error) / DT;

    ct->control_candidate = (ct->Kp * ct->error) + (ct->Ki * ct->integral_candidate) + (ct->Kd * error_deriv);
    if(ct->control_candidate > ct->max)
    {
        ct->control = ct->max;
    }
    else if(ct->control_candidate < ct->min)
    {
        ct->control = ct->min;
    }
    else
    {
        ct->control = ct->control_candidate;
        ct->integral = ct->integral_candidate;
    }
    return ct->control;
}

float wheelFeedForward(float target_wheel_speed)
{
    return target_wheel_speed * (KN + (DAMPING * RESISTANCE_ohms) / KM);   
}

void updateControl(void)
{
    for(Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        wheel_speed_pid[wheel].target = wheel_speed_targets[wheel];
        float measurement = wheelSpeed_get(wheel);
        float pid_output = PIDController(&wheel_speed_pid[wheel], measurement);
        float ff_output = wheelFeedForward(wheel_speed_targets[wheel]);
        float voltage = pid_output + ff_output;
        motorDriver_setVoltage(wheel, voltage);
    }
}
