#include "sharedFunctions.h"
#include "wheelSpeeds.h"
#include "imu.h"

#include "esp_timer.h"
#include "math.h"

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
static float sliding_accel_sum_squared = 0.f;
static float accel_1 = 0.f;
static float accel_2 = 0.f;
static float accel_3 = 0.f;
static float accel_4 = 0.f;

static float final_velcity_estimate = 0.f;
static float vehicle_velcocity_prediction = 0.f;
static float velocity_lamda = 1.f;
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
    
    accel_1 = accelValues.ax;
    sliding_accel_sum_squared = (accel_1 * accel_1 + accel_2 * accel_2 + accel_3 * accel_3 + accel_4 * accel_4);

    // changing kalman parameters based on the accel integrated vehicle speed
    // prediciton
    vehicle_velcocity_prediction += time_elapsed * accelValues.ax * 9.81f;
    vehicle_Estimates.vehicle_speed_estimate += time_elapsed * accelValues.ax * 9.81f;

    // vehicle is stopped
    if ((vehicle_Estimates.vehicle_speed_estimate < 0.f) || ((averaged_wheel_speed == 0.f) && (sliding_accel_sum_squared < SLIDING_ACCEL_MAX)) || (sliding_accel_sum_squared < SLIDING_ACCEL_MAX))
    {
        vehicle_velcocity_prediction = 0.f;
        vehicle_Estimates.vehicle_speed_estimate = 0.f;
    }  
    vehicle_Kalman_Parameters.error_covariance += Q_ACCEL;

    {    
    // measurement  update
    vehicle_Kalman_Parameters.innovation_covariance = vehicle_Kalman_Parameters.error_covariance + R_WHEEL_SPEEDS;
    vehicle_Kalman_Parameters.kalman_gain = vehicle_Kalman_Parameters.error_covariance / vehicle_Kalman_Parameters.innovation_covariance;
    vehicle_Estimates.vehicle_speed_estimate += vehicle_Kalman_Parameters.kalman_gain * (averaged_wheel_speed * WHEEL_RADIUS - vehicle_Estimates.vehicle_speed_estimate);
    vehicle_Kalman_Parameters.error_covariance -= vehicle_Kalman_Parameters.kalman_gain * vehicle_Kalman_Parameters.error_covariance;
    
    if (sliding_accel_sum_squared < SLIDING_ACCEL_MAX)
    {
        velocity_lamda += 0.005f;
    }
    else
    {
        velocity_lamda -= 0.005f;
    }

    if (velocity_lamda > 1.f) velocity_lamda = 1.f;
    if (velocity_lamda < 0.f) velocity_lamda = 0.f;
    }
    final_velcity_estimate = velocity_lamda * vehicle_velcocity_prediction + (1.f - velocity_lamda) * vehicle_Estimates.vehicle_speed_estimate;
    // final_velcity_estimate = vehicle_Estimates.vehicle_speed_estimate;

    accel_2 = accel_1;
    accel_3 = accel_2;
    accel_4 = accel_3;
}

float get_vehicle_velocity_estimate_KF(void)
{
    return final_velcity_estimate;
}