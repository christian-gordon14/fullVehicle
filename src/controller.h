#ifndef CONTROLLER_H
#define CONTROLLER_H

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