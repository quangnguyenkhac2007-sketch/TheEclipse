#include "pid.h"

#include <math.h>
#include <stddef.h>

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

bool PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if ((pid == NULL) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(output_min) || !isfinite(output_max) ||
        (output_min >= output_max))
    {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    PID_Reset(pid);
    return true;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = false;
}

bool PID_Compute(PID_Controller *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds,
                 float *output)
{
    float error;
    float derivative;
    float candidate_integral;
    float candidate_output;

    if ((pid == NULL) || (output == NULL) ||
        !isfinite(setpoint) || !isfinite(measurement) ||
        !isfinite(dt_seconds) || (dt_seconds <= 0.0f))
    {
        return false;
    }

    error = setpoint - measurement;
    candidate_integral = pid->integral + (error * dt_seconds);
    derivative = pid->has_previous_error
                     ? (error - pid->previous_error) / dt_seconds
                     : 0.0f;
    candidate_output = (pid->kp * error) +
                       (pid->ki * candidate_integral) +
                       (pid->kd * derivative);

    if (!isfinite(candidate_output))
    {
        return false;
    }

    *output = Clamp(candidate_output, pid->output_min, pid->output_max);

    /* Do not accumulate integral error while the output is saturated. */
    if ((candidate_output == *output) ||
        ((candidate_output > pid->output_max) && (error < 0.0f)) ||
        ((candidate_output < pid->output_min) && (error > 0.0f)))
    {
        pid->integral = candidate_integral;
    }

    pid->previous_error = error;
    pid->has_previous_error = true;
    return true;
}