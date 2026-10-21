#include "bluetooth.h"

#include <stddef.h>

static UART_HandleTypeDef *bluetooth_uart;

bool Bluetooth_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return false;
    }

    bluetooth_uart = uart;
    return true;
}

bool Bluetooth_Send(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    if ((bluetooth_uart == NULL) || (data == NULL) || (length == 0U) ||
        (length > UINT16_MAX))
    {
        return false;
    }

    return HAL_UART_Transmit(bluetooth_uart,
                             (uint8_t *)data,
                             (uint16_t)length,
                             timeout_ms) == HAL_OK;
}

bool Bluetooth_Receive(uint8_t *data, size_t length, uint32_t timeout_ms)
{
    if ((bluetooth_uart == NULL) || (data == NULL) || (length == 0U) ||
        (length > UINT16_MAX))
    {
        return false;
    }

    return HAL_UART_Receive(bluetooth_uart,
                            data,
                            (uint16_t)length,
                            timeout_ms) == HAL_OK;
}
