#include "control.h"

#include "config.h"
#include "motor.h"
#include "pid.h"
#include "sensor.h"

static PID_Controller line_pid;

bool Control_Init(const Sensor_Config *sensor_config,
                  const Motor_Config *motor_config)
{
    if (!PID_Init(&line_pid,
                  LINE_FOLLOWER_PID_KP,
                  LINE_FOLLOWER_PID_KI,
                  LINE_FOLLOWER_PID_KD,
                  LINE_FOLLOWER_PID_OUTPUT_MIN,
                  LINE_FOLLOWER_PID_OUTPUT_MAX))
    {
        return false;
    }

    if (!Motor_Init(motor_config))
    {
        Motor_Stop();
        return false;
    }

    if (!Sensor_Init(sensor_config))
    {
        Motor_Stop();
        return false;
    }

    return true;
}

bool Control_Update(float dt_seconds)
{
    float line_position;
    float correction;
    float left_speed;
    float right_speed;

    if (!Sensor_ReadLinePosition(&line_position) ||
        !PID_Compute(&line_pid, 0.0f, line_position, dt_seconds, &correction))
    {
        Motor_Stop();
        return false;
    }

    left_speed = LINE_FOLLOWER_BASE_SPEED - correction;
    right_speed = LINE_FOLLOWER_BASE_SPEED + correction;

    if (!Motor_SetSpeed(left_speed, right_speed))
    {
        Motor_Stop();
        return false;
    }

    return true;
}

void Control_Stop(void)
{
    Motor_Stop();
    PID_Reset(&line_pid);
}