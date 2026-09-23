#ifndef VSE_H
#define VSE_H

#include "wheelSpeeds.h"
#include "vehicleDynamics.h"

typedef struct {
    float vehicle_speed_estimate;
} Vehicle_Estimates;

typedef struct {
    float estimate;
} Wheel_Speed_Estimates;

typedef struct {
    float error_covariance;
    float innovation_covariance;
    float kalman_gain;
} Kalman_Parameters;

// vehicle speed estimation KF
#define Q_ACCEL 1.71603f
#define R_WHEEL_SPEEDS_LINEAR 0.07227f

// wheel speed estimation KF
#define Q_WHEEL_SPEEDS 1.f
#define R_WHEEL_SPEEDS 0.07227f

// vehicle parameters
#define VEHICLE_MASS 4.f
#define WHEEL_RADIUS 0.032f
#define WHEEL_INERTIA 1.9345E-5f
#define KM 0.275f
#define KN 0.250f
#define WHEEL_DAMPING 0.001f


// misc
#define SLIDING_ACCEL_MAX 3e-5

// function definitions
float get_model_wheel_speeds(Wheel wheel);
float get_vehicle_velocity_estimate_KF(void);
float get_wheel_speeds_estimate_KF(Wheel wheel);
void call_VSE(void);

// LUT
static const float wheel_speed_LUT[] = 
{
    0.f,
    1.f,
    2.f,
    3.f,
    4.f,
    5.f 
};

static const float surface_force_LUT[] = 
{
    0.f,
    0.5f,
    1.0f,
    1.5f,
    2.0f,
    5.f,
};

#endif