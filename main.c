#include "main.h"

#include "board.h"
#include "config.h"
#include "control.h"

int main(void)
{
    uint32_t last_control_tick;

    HAL_Init();
    if (!Board_Init())
    {
        Error_Handler();
    }

    if (!Control_Init(Board_GetSensorConfig(), Board_GetMotorConfig()))
    {
        Error_Handler();
    }

    last_control_tick = HAL_GetTick();
    for (;;)
    {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_control_tick) >=
            (uint32_t)(LINE_FOLLOWER_CONTROL_PERIOD_S * 1000.0f))
        {
            last_control_tick = now;
            if (!Control_Update(LINE_FOLLOWER_CONTROL_PERIOD_S))
            {
                Error_Handler();
            }
        }
    }
}

void Error_Handler(void)
{
    Control_Stop();

    for (;;)
    {
        /* Stay stopped until the fault is handled or the MCU is reset. */
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}