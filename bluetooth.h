#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool Bluetooth_Init(UART_HandleTypeDef *uart);
bool Bluetooth_Send(const uint8_t *data, size_t length, uint32_t timeout_ms);
bool Bluetooth_Receive(uint8_t *data, size_t length, uint32_t timeout_ms);

#endif /* BLUETOOTH_H */
