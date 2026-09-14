#include "sharedFunctions.h"
#include "wheelSpeeds.h"
#include "imu.h"

#include "esp_timer.h"

// ============================================================
// Configuration
// ============================================================

// ============================================================
// Function definitions
// ============================================================

// ============================================================
// Private variables
// ============================================================
Vehicle_Estimates vehicle_Estimates = {0};
Kalman_Parameters vehicle_Kalman_Parameters = {0};
static float averaged_wheel_speed = 0.f;
static AccelValues accelValues;

// ============================================================
// Public variables
// ============================================================
static float time_elapsed = 0.001f;

// ============================================================
// Private functions
// ============================================================


// ============================================================
// Public functions
// ============================================================

void kalmanFilter(void)
{
    static int64_t last_time = 0;
    int64_t start_time = esp_timer_get_time();

    if (last_time != 0)
    {
        time_elapsed = (start_time - last_time) * 1e-6f; 
    }
    
    last_time = start_time;

    accelValues = imu_get_accel();
    averaged_wheel_speed = get_middle_wheel_speeds_average();

    // prediciton
    vehicle_Estimates.vehicle_speed_estimate += time_elapsed * accelValues.ax * 9.81f;
    vehicle_Kalman_Parameters.error_covariance += Q_ACCEL;
    
    // measurement  update
    vehicle_Kalman_Parameters.innovation_covariance = vehicle_Kalman_Parameters.error_covariance + R_WHEEL_SPEEDS;
    vehicle_Kalman_Parameters.kalman_gain = vehicle_Kalman_Parameters.error_covariance / vehicle_Kalman_Parameters.innovation_covariance;
    vehicle_Estimates.vehicle_speed_estimate += vehicle_Kalman_Parameters.kalman_gain * (averaged_wheel_speed * WHEEL_RADIUS - vehicle_Estimates.vehicle_speed_estimate);
    vehicle_Kalman_Parameters.error_covariance -= vehicle_Kalman_Parameters.kalman_gain * vehicle_Kalman_Parameters.error_covariance;

}

float get_vehicle_velocity_estimate_KF(void)
{
    return vehicle_Estimates.vehicle_speed_estimate;
}