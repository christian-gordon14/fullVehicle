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

float PIDController(PID_params *controller, float measurement);
float wheelFeedForward(float target_wheel_speed);
void updateControl(void);
#endif