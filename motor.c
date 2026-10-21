#include "motor.h"

#include <math.h>
#include <stddef.h>

static Motor_Config motor_config;
static bool motor_initialized;

static bool IsValidConfig(const Motor_Config *config)
{
    return (config != NULL) &&
           (config->left_in1_port != NULL) && (config->left_in1_pin != 0U) &&
           (config->left_in2_port != NULL) && (config->left_in2_pin != 0U) &&
           (config->right_in1_port != NULL) && (config->right_in1_pin != 0U) &&
           (config->right_in2_port != NULL) && (config->right_in2_pin != 0U) &&
           (config->pwm_timer != NULL) &&
           (__HAL_TIM_GET_AUTORELOAD(config->pwm_timer) > 0U) &&
           (config->left_pwm_channel != config->right_pwm_channel);
}

static void SetDuty(uint32_t channel, float magnitude_percent)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor_config.pwm_timer);
    uint32_t compare =
        (uint32_t)(((float)(period + 1U) * magnitude_percent) / 100.0f);

    __HAL_TIM_SET_COMPARE(motor_config.pwm_timer, channel, compare);
}

bool Motor_Init(const Motor_Config *config)
{
    motor_initialized = false;
    Motor_Stop();

    if (!IsValidConfig(config))
    {
        return false;
    }

    motor_config = *config;

    if ((motor_config.standby_port != NULL) &&
        (motor_config.standby_pin != 0U))
    {
        HAL_GPIO_WritePin(motor_config.standby_port,
                          motor_config.standby_pin,
                          GPIO_PIN_RESET);
    }
    SetDuty(motor_config.left_pwm_channel, 0.0f);
    SetDuty(motor_config.right_pwm_channel, 0.0f);

    if ((HAL_TIM_PWM_Start(motor_config.pwm_timer,
                           motor_config.left_pwm_channel) != HAL_OK) ||
        (HAL_TIM_PWM_Start(motor_config.pwm_timer,
                           motor_config.right_pwm_channel) != HAL_OK))
    {
        Motor_Stop();
        return false;
    }

    motor_initialized = true;
    return true;
}

bool Motor_SetSpeed(float left_percent, float right_percent)
{
    GPIO_PinState left_forward;
    GPIO_PinState right_forward;
    float left_magnitude;
    float right_magnitude;

    if (!motor_initialized || !isfinite(left_percent) ||
        !isfinite(right_percent))
    {
        return false;
    }

    if (left_percent > 100.0f)
    {
        left_percent = 100.0f;
    }
    else if (left_percent < -100.0f)
    {
        left_percent = -100.0f;
    }
    if (right_percent > 100.0f)
    {
        right_percent = 100.0f;
    }
    else if (right_percent < -100.0f)
    {
        right_percent = -100.0f;
    }

    if (motor_config.left_motor_reversed)
    {
        left_percent = -left_percent;
    }
    if (motor_config.right_motor_reversed)
    {
        right_percent = -right_percent;
    }

    left_forward = (left_percent >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    right_forward = (right_percent >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    left_magnitude = (left_percent < 0.0f) ? -left_percent : left_percent;
    right_magnitude = (right_percent < 0.0f) ? -right_percent : right_percent;

    HAL_GPIO_WritePin(motor_config.left_in1_port,
                      motor_config.left_in1_pin,
                      left_forward);
    HAL_GPIO_WritePin(motor_config.left_in2_port,
                      motor_config.left_in2_pin,
                      (left_forward == GPIO_PIN_SET)
                          ? GPIO_PIN_RESET
                          : GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_config.right_in1_port,
                      motor_config.right_in1_pin,
                      right_forward);
    HAL_GPIO_WritePin(motor_config.right_in2_port,
                      motor_config.right_in2_pin,
                      (right_forward == GPIO_PIN_SET)
                          ? GPIO_PIN_RESET
                          : GPIO_PIN_SET);

    SetDuty(motor_config.left_pwm_channel, left_magnitude);
    SetDuty(motor_config.right_pwm_channel, right_magnitude);
    if ((motor_config.standby_port != NULL) &&
        (motor_config.standby_pin != 0U))
    {
        HAL_GPIO_WritePin(motor_config.standby_port,
                          motor_config.standby_pin,
                          GPIO_PIN_SET);
    }
    return true;
}

void Motor_Stop(void)
{
    motor_initialized = false;

    if ((motor_config.standby_port != NULL) &&
        (motor_config.standby_pin != 0U))
    {
        HAL_GPIO_WritePin(motor_config.standby_port,
                          motor_config.standby_pin,
                          GPIO_PIN_RESET);
    }
    if (motor_config.pwm_timer != NULL)
    {
        SetDuty(motor_config.left_pwm_channel, 0.0f);
        SetDuty(motor_config.right_pwm_channel, 0.0f);
    }
}