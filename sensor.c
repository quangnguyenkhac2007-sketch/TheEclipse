#include "sensor.h"

#include "config.h"

#include <stddef.h>

static Sensor_Config sensor_config;
static bool sensor_initialized;
static const float position_weights[LINE_SENSOR_COUNT] =
    {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

static bool IsValidConfig(const Sensor_Config *config)
{
    uint32_t i;

    if ((config == NULL) ||
        ((config->mode != SENSOR_INPUT_DIGITAL) &&
         (config->mode != SENSOR_INPUT_ANALOG)))
    {
        return false;
    }

    if (config->mode == SENSOR_INPUT_ANALOG)
    {
        if (config->adc == NULL)
        {
            return false;
        }
        for (i = 0U; i < LINE_SENSOR_COUNT; ++i)
        {
            if ((config->analog_thresholds[i] == 0U) ||
                (config->analog_thresholds[i] >= LINE_SENSOR_ADC_MAX))
            {
                return false;
            }
        }
    }
    else
    {
        for (i = 0U; i < LINE_SENSOR_COUNT; ++i)
        {
            if ((config->gpio_ports[i] == NULL) ||
                (config->gpio_pins[i] == 0U))
            {
                return false;
            }
        }
    }

    return true;
}

bool Sensor_Init(const Sensor_Config *config)
{
    sensor_initialized = false;
    if (!IsValidConfig(config))
    {
        return false;
    }

    sensor_config = *config;
    sensor_initialized = true;
    return true;
}

static bool ReadAnalog(uint32_t index, uint32_t *value)
{
    ADC_ChannelConfTypeDef channel_config = {0};
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef stop_status;

    channel_config.Channel = sensor_config.adc_channels[index];
    channel_config.Rank = ADC_REGULAR_RANK_1;
    channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

    status = HAL_ADC_ConfigChannel(sensor_config.adc, &channel_config);
    if (status != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start(sensor_config.adc) != HAL_OK)
    {
        return false;
    }

    status = HAL_ADC_PollForConversion(sensor_config.adc, 2U);
    if (status == HAL_OK)
    {
        *value = HAL_ADC_GetValue(sensor_config.adc);
    }

    stop_status = HAL_ADC_Stop(sensor_config.adc);
    return (status == HAL_OK) && (stop_status == HAL_OK);
}

bool Sensor_ReadLinePosition(float *position)
{
    float weighted_sum = 0.0f;
    float strength_sum = 0.0f;
    uint32_t i;

    if (!sensor_initialized || (position == NULL))
    {
        return false;
    }

    for (i = 0U; i < LINE_SENSOR_COUNT; ++i)
    {
        float strength;

        if (sensor_config.mode == SENSOR_INPUT_DIGITAL)
        {
            GPIO_PinState state =
                HAL_GPIO_ReadPin(sensor_config.gpio_ports[i],
                                 sensor_config.gpio_pins[i]);
            bool active = sensor_config.line_is_low
                              ? (state == GPIO_PIN_RESET)
                              : (state == GPIO_PIN_SET);
            strength = active ? 1.0f : 0.0f;
        }
        else
        {
            uint32_t raw_value;
            uint32_t threshold = sensor_config.analog_thresholds[i];

            if (!ReadAnalog(i, &raw_value))
            {
                return false;
            }

            if (sensor_config.line_is_low)
            {
                strength = (raw_value < threshold)
                               ? (float)(threshold - raw_value) /
                                     (float)threshold
                               : 0.0f;
            }
            else
            {
                strength = (raw_value > threshold)
                               ? (float)(raw_value - threshold) /
                                     (float)(LINE_SENSOR_ADC_MAX - threshold)
                               : 0.0f;
            }
        }

        weighted_sum += position_weights[i] * strength;
        strength_sum += strength;
    }

    if (strength_sum <= 0.0f)
    {
        return false;
    }

    *position = weighted_sum / strength_sum;
    return true;
}