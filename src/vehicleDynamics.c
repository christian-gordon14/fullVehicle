#include "vehicleDynamics.h"
#include "imu.h"
#include "wheelSpeeds.h"

#include <math.h>
// ============================================================
// Configuration
// ============================================================
#define VEHICLE_MASS 0.75f
#define GRAVITY 9.81f
#define MU 1.0f
#define WHEEL_RADIUS 0.032f

// ============================================================
// Function Declaration
// ============================================================
static float calcSlipRatio(float linearWheelSpeed, float vehicleSpeed);
static float pacejkaTireModel(float slipAngle);
static void wheelForceCalc(void);

// ============================================================
// Private variables
// ============================================================


// ============================================================
// Public variables
// ============================================================

VehicleStates vehicleStates;
WheelSpeed wheelSpeed;
WheelStruct slipRatios;
WheelStruct wheelForces;
AccelValues accelValues;

// ============================================================
// Private functions
// ============================================================

static float calcSlipRatio(float linearWheelSpeed,float vehicleSpeed)
{
    accelValues = imu_get_accel();
    if (accelValues.ax > 0.f)
    {
        if (linearWheelSpeed != 0.f)
        {
            return (linearWheelSpeed - vehicleSpeed) / linearWheelSpeed;
        }
        return 0.f;
        
    }
    else if (accelValues.ax < 0.f)
    {
        if (vehicleSpeed != 0.f)
        {
            return (linearWheelSpeed - vehicleSpeed) / vehicleSpeed;
        }
        return 0.f;
    }
    else
    {
        return 0.;
    }
}

static float pacejkaTireModel(float slip)
{
    float D = VEHICLE_MASS * GRAVITY * MU;
    float B = 10.f;
    float C = 2.3f;
    float E = 0.67f;

    float Fx = D * sin(C * atan(B * slip - E * (B * slip - atan(B * slip))));
    return Fx;
}

static void wheelForceCalc(void)
{
    float *forces = &wheelForces.WHEEL_FL;
    float *slip = &slipRatios.WHEEL_FL;

    for (Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        float vehicleSpeed = vehicleStates.xVelocity;
        float linearWheelSpeed = wheelSpeed_get(wheel) * WHEEL_RADIUS;
        slip[wheel] = calcSlipRatio(linearWheelSpeed, vehicleSpeed);
        forces[wheel] = pacejkaTireModel(slip[wheel]);
    }
}

// ============================================================
// Public functions
// ============================================================

WheelStruct getWheelForces(void)
{
    wheelForceCalc();
    return wheelForces;
}

WheelStruct getSlipRatios(void)
{
    wheelForceCalc();
    return slipRatios;
}