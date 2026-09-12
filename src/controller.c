#include "controller.h"
#include "wheelSpeeds.h"
#include "motorDriver.h"
#include "bluetooth.h"
#include "imu.h"
#include "vehicleDynamics.h"
#include <stdio.h>

// ============================================================
// Configuration
// ============================================================
#define DT 0.002f
#define RESISTANCE_ohms 6.4f
#define DAMPING 0.001f
#define KM 0.275f
#define KN 0.2540f

// #define KP_WHEEL_SPEED 0.8f
// #define KI_WHEEL_SPEED 1.28f
// tuning
#define KP_WHEEL_SPEED 0.05f
#define KI_WHEEL_SPEED 1.2f
#define KD_WHEEL_SPEED 0.0f

#define KP_STABILITY 1.f
#define MAX_YAW_OUTPUT 3.f
#define MIN_YAW_OUTPUT -3.f
#define TARGET_HEADING 0.0f

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

static PID_params yaw_heading_pid = {
    .Kp = KP_STABILITY,
    .Ki = 0,
    .Kd = 0,
    .max = MAX_YAW_OUTPUT,
    .min = MIN_YAW_OUTPUT,
    .target = TARGET_HEADING,
};

// ============================================================
// Public variables
// ============================================================
Controller_outputs controller_outputs;
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

Controller_outputs updateControl(void)
{
    float target = writeSpeed() ? 13.f : 0.f;
    // printf("target = %.2f, writeSpeed = %d\n", target, writeSpeed());
    VehicleStates vehicleStates = imu_get_states();
    float yaw_heading_pid_output = PIDController(&yaw_heading_pid, vehicleStates.heading);
    float ff_output = wheelFeedForward(target);

    // iterate through all wheels
    for(Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        // wheel speed
        wheel_speed_pid[wheel].target = target;        
        float wheel_speed_measurement = wheelSpeed_get(wheel);
        float pid_output = PIDController(&wheel_speed_pid[wheel], wheel_speed_measurement);
        // heading
        float scale = (wheel == WHEEL_FL || wheel == WHEEL_RL) ? -1.0f : 1.0f;

        // sum and setting motors
        float voltage = pid_output + ff_output + scale * yaw_heading_pid_output;
        motorDriver_setVoltage(wheel, voltage);

        // returning controls
        switch (wheel)
        {
        case WHEEL_FL:
            controller_outputs.PID_wheel_speed_FL = pid_output;
            break;
        case WHEEL_FR:
            controller_outputs.PID_wheel_speed_FR = pid_output;
            break;
        case WHEEL_RL:
            controller_outputs.PID_wheel_speed_RL = pid_output;
            break;
        case WHEEL_RR:
            controller_outputs.PID_wheel_speed_RR = pid_output;
            break;
        default:
            break;
        }
    }
    controller_outputs.feedforward = ff_output;
    controller_outputs.PID_heading = yaw_heading_pid_output;
    return controller_outputs;
}
