#ifndef SHARED_FUNCTIONS_H
#define SHARED_FUNCTIONS_H

typedef struct {
    float vehicle_speed_estimate;
} Vehicle_Estimates;

typedef struct {
    float error_covariance;
    float innovation_covariance;
    float kalman_gain;
} Kalman_Parameters;

#define Q_ACCEL 1.71603f
#define R_WHEEL_SPEEDS 0.07227f

#define WHEEL_RADIUS 0.032f

void kalmanFilter(void);
float get_vehicle_velocity_estimate_KF(void);

#endif