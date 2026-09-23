#include "vehicleStateEstimation.h"
#include "wheelSpeeds.h"
#include "imu.h"
#include "controller.h"

#include "esp_timer.h"
#include "math.h"

// ============================================================
// Configuration
// ============================================================

// ============================================================
// Function definitions
// ============================================================
static void kalmanFilter_velocityEstimate(void);
static void kalmanFilter_wheelSpeedsEstimate(void);
// ============================================================
// Private variables
// ============================================================
Vehicle_Estimates vehicle_Estimates = {0};
Wheel_Speed_Estimates wheel_Speed_Estimates[WHEEL_COUNT] = {0};

Kalman_Parameters vehicle_estimate_Kalman_Parameters = {0};
Kalman_Parameters wheel_speed_Kalman_Parameters[WHEEL_COUNT] = {0};
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

static float model_wheel_speeds[WHEEL_COUNT] = {0};
static float measured_wheel_speeds[WHEEL_COUNT] = {0};
// ============================================================
// Public variables
// ============================================================

// ============================================================
// Private functions
// ============================================================
static void wheelSpeedModel(void)
{
    static int64_t last_time = 0;
    int64_t start_time = esp_timer_get_time();

    if (last_time == 0)
    {
        last_time = start_time;
        return;
    }

    float dt = (start_time - last_time) * 1e-6f;
    last_time = start_time;

    float *Vs = get_voltage_output();
    // float F_surface = 6.0f; 
    float F_surface = 3.f; 

    for (Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        model_wheel_speeds[wheel] = (model_wheel_speeds[wheel] + dt / WHEEL_INERTIA * (KM * Vs[wheel] / RESISTANCE_ohms - F_surface * WHEEL_RADIUS)) / 
                                    (1.f + dt / WHEEL_INERTIA * (KM * KN / RESISTANCE_ohms + WHEEL_DAMPING));
        if (model_wheel_speeds[wheel] <= 0.f)
        {
            model_wheel_speeds[wheel] = 0.f;
        }
    }
}

static void kalmanFilter_wheelSpeedsEstimate(void)
{
    static int64_t last_time = 0;
    int64_t start_time = esp_timer_get_time();

    if (last_time == 0)
    {
        last_time = start_time;
        return;
    }

    float dt = (start_time - last_time) * 1e-6f;
    last_time = start_time;

    for(Wheel wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
        measured_wheel_speeds[wheel] = wheelSpeed_get(wheel);

        // prediction
        wheel_Speed_Estimates[wheel].estimate = get_model_wheel_speeds(wheel);
        wheel_speed_Kalman_Parameters[wheel].error_covariance += Q_WHEEL_SPEEDS * dt;
        
        // correction
        wheel_speed_Kalman_Parameters[wheel].innovation_covariance = wheel_speed_Kalman_Parameters[wheel].error_covariance + R_WHEEL_SPEEDS;
        wheel_speed_Kalman_Parameters[wheel].kalman_gain = wheel_speed_Kalman_Parameters[wheel].error_covariance / wheel_speed_Kalman_Parameters[wheel].innovation_covariance;
        wheel_Speed_Estimates[wheel].estimate += wheel_speed_Kalman_Parameters[wheel].kalman_gain * (measured_wheel_speeds[wheel] - wheel_Speed_Estimates[wheel].estimate);
        wheel_speed_Kalman_Parameters[wheel].error_covariance -= wheel_speed_Kalman_Parameters[wheel].kalman_gain * wheel_speed_Kalman_Parameters[wheel].error_covariance;

    }
}

static void kalmanFilter_velocityEstimate(void)
{
    static int64_t last_time = 0;
    int64_t start_time = esp_timer_get_time();

    if (last_time == 0)
    {
        last_time = start_time;
        return;
    }

    float dt = (start_time - last_time) * 1e-6f;
    last_time = start_time;

    accelValues = imu_get_accel();
    averaged_wheel_speed = get_middle_wheel_speeds_average();
    // averaged_wheel_speed =
    // 0.25f *
    // (wheel_Speed_Estimates[WHEEL_FL].estimate +
    //  wheel_Speed_Estimates[WHEEL_FR].estimate +
    //  wheel_Speed_Estimates[WHEEL_RL].estimate +
    //  wheel_Speed_Estimates[WHEEL_RR].estimate);
    
    accel_1 = accelValues.ax;
    sliding_accel_sum_squared = (accel_1 * accel_1 + accel_2 * accel_2 + accel_3 * accel_3 + accel_4 * accel_4);

    // changing kalman parameters based on the accel integrated vehicle speed
    // prediciton
    vehicle_velcocity_prediction += dt * accelValues.ax * 9.81f;
    vehicle_Estimates.vehicle_speed_estimate += dt * accelValues.ax * 9.81f;

    // vehicle is stopped
    if ((vehicle_Estimates.vehicle_speed_estimate < 0.f) || ((averaged_wheel_speed == 0.f) && (sliding_accel_sum_squared < SLIDING_ACCEL_MAX)) || (sliding_accel_sum_squared < SLIDING_ACCEL_MAX))
    {
        vehicle_velcocity_prediction = 0.f;
        vehicle_Estimates.vehicle_speed_estimate = 0.f;
    }  
    vehicle_estimate_Kalman_Parameters.error_covariance += Q_ACCEL * dt;

    {    
    // measurement  update
    vehicle_estimate_Kalman_Parameters.innovation_covariance = vehicle_estimate_Kalman_Parameters.error_covariance + R_WHEEL_SPEEDS_LINEAR;
    vehicle_estimate_Kalman_Parameters.kalman_gain = vehicle_estimate_Kalman_Parameters.error_covariance / vehicle_estimate_Kalman_Parameters.innovation_covariance;
    vehicle_Estimates.vehicle_speed_estimate += vehicle_estimate_Kalman_Parameters.kalman_gain * (averaged_wheel_speed * WHEEL_RADIUS - vehicle_Estimates.vehicle_speed_estimate);
    vehicle_estimate_Kalman_Parameters.error_covariance -= vehicle_estimate_Kalman_Parameters.kalman_gain * vehicle_estimate_Kalman_Parameters.error_covariance;
    
    if (sliding_accel_sum_squared < SLIDING_ACCEL_MAX)
    {
        velocity_lamda -= 0.005f;
    }
    else
    {
        velocity_lamda += 0.005f;
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

// ============================================================
// Public functions
// ============================================================

float get_model_wheel_speeds(Wheel wheel)
{
    if(wheel >= WHEEL_COUNT)
    {
        return 0.f;
    }
    return model_wheel_speeds[wheel];
}


float get_vehicle_velocity_estimate_KF(void)
{
    return final_velcity_estimate;
}


float get_wheel_speeds_estimate_KF(Wheel wheel)
{
    if(wheel >= WHEEL_COUNT)
    {
        return 0.f;
    }
    return wheel_Speed_Estimates[wheel].estimate;
}

void call_VSE(void)
{
    wheelSpeedModel();
    kalmanFilter_wheelSpeedsEstimate();
    kalmanFilter_velocityEstimate();
}