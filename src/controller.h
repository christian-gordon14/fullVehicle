#ifndef CONTROLLER_H
#define CONTROLLER_H
#define DT 0.002f
#define RESISTANCE_ohms 6.4f
#define DAMPING 0.001f
#define KM 0.275f
#define KN 0.2540f
#define VEHICLE_FRICTION 0.3f


// #define KP_WHEEL_SPEED 0.8f
// #define KI_WHEEL_SPEED 1.28f
// tuning
#define KP_WHEEL_SPEED 0.01f
#define KI_WHEEL_SPEED 4.3f
#define KD_WHEEL_SPEED 0.0f

#define KP_STABILITY 1.f
#define MAX_YAW_OUTPUT 3.f
#define MIN_YAW_OUTPUT -3.f
#define TARGET_HEADING 0.0f

#define TARGET_WHEEL_SPEED 11.0f

typedef struct{
    float error;
    float target;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float integral_candidate;
    float control;
    float control_candidate;
    float max;
    float min;
} PID_params;

typedef struct {
    float feedforward;
    float PID_wheel_speed_FL;
    float PID_wheel_speed_FR;
    float PID_wheel_speed_RL;
    float PID_wheel_speed_RR;
    float PID_heading;
} Controller_outputs;

float PIDController(PID_params *controller, float measurement);
float wheelFeedForward(float target_wheel_speed);
Controller_outputs updateControl(void);
#endif