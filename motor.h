#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *left_in1_port;
    uint16_t left_in1_pin;
    GPIO_TypeDef *left_in2_port;
    uint16_t left_in2_pin;
    GPIO_TypeDef *right_in1_port;
    uint16_t right_in1_pin;
    GPIO_TypeDef *right_in2_port;
    uint16_t right_in2_pin;
    GPIO_TypeDef *standby_port;
    uint16_t standby_pin;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t left_pwm_channel;
    uint32_t right_pwm_channel;
    bool left_motor_reversed;
    bool right_motor_reversed;
} Motor_Config;

bool Motor_Init(const Motor_Config *config);

/* Set each motor speed in the range [-100.0, 100.0]. */
bool Motor_SetSpeed(float left_percent, float right_percent);

void Motor_Stop(void);

#endif /* MOTOR_H */