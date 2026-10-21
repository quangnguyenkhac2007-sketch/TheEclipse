#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SENSOR_INPUT_DIGITAL = 0,
    SENSOR_INPUT_ANALOG
} Sensor_InputMode;

typedef struct
{
    Sensor_InputMode mode;
    bool line_is_low;
    /* Entries must be ordered from the leftmost sensor to the rightmost. */
    GPIO_TypeDef *gpio_ports[LINE_SENSOR_COUNT];
    uint16_t gpio_pins[LINE_SENSOR_COUNT];
    ADC_HandleTypeDef *adc;
    uint32_t adc_channels[LINE_SENSOR_COUNT];
    uint16_t analog_thresholds[LINE_SENSOR_COUNT];
} Sensor_Config;

bool Sensor_Init(const Sensor_Config *config);

/*
 * Read the line position normalized to [-1.0, 1.0].
 * Return false if the sensor cannot be read, the line is not detected, or
 * the ADC/GPIO operation fails.
 */
bool Sensor_ReadLinePosition(float *position);

#endif /* SENSOR_H */