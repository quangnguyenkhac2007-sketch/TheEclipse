#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral;
    float previous_error;
    bool has_previous_error;
} PID_Controller;

bool PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);
void PID_Reset(PID_Controller *pid);
bool PID_Compute(PID_Controller *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds,
                 float *output);

#endif /* PID_H */