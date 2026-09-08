#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H

typedef struct {
    float WHEEL_FL;
    float WHEEL_FR;
    float WHEEL_RL;
    float WHEEL_RR;
} WheelStruct;

WheelStruct getSlipRatios(void);
WheelStruct getWheelForces(void);
#endif