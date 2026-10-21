#include "board.h"

#include "bluetooth.h"
#include "config.h"
#include "mpu6050.h"

#include <stddef.h>

static ADC_HandleTypeDef hadc1;
static I2C_HandleTypeDef hi2c1;
static TIM_HandleTypeDef htim1;
static UART_HandleTypeDef huart1;

static Sensor_Config sensor_config = {
    .mode = SENSOR_INPUT_ANALOG,
    /* Analog thresholding is configured per channel; tune polarity/thresholds
     * after measuring the actual sensor voltages on the track. */
    .line_is_low = true,
    .gpio_ports = {NULL, NULL, NULL, NULL, NULL},
    .gpio_pins = {0U, 0U, 0U, 0U, 0U},
    .adc = &hadc1,
    .adc_channels = {
        ADC_CHANNEL_2,
        ADC_CHANNEL_3,
        ADC_CHANNEL_4,
        ADC_CHANNEL_5,
        ADC_CHANNEL_6
    },
    .analog_thresholds = {2048U, 2048U, 2048U, 2048U, 2048U}
};

static Motor_Config motor_config = {
    .left_in1_port = GPIOB,
    .left_in1_pin = GPIO_PIN_12,
    .left_in2_port = GPIOB,
    .left_in2_pin = GPIO_PIN_13,
    .right_in1_port = GPIOB,
    .right_in1_pin = GPIO_PIN_14,
    .right_in2_port = GPIOB,
    .right_in2_pin = GPIO_PIN_15,
    .standby_port = NULL,
    .standby_pin = 0U,
    .pwm_timer = &htim1,
    .left_pwm_channel = TIM_CHANNEL_1,
    .right_pwm_channel = TIM_CHANNEL_4,
    .left_motor_reversed = false,
    .right_motor_reversed = false
};

static bool ConfigureClock(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        return false;
    }

    clock.ClockType = RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 |
                      RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0) == HAL_OK;
}

static void ConfigureGpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
               GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    __HAL_AFIO_REMAP_I2C1_ENABLE();
}

static bool ConfigureAdc(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;

    __HAL_RCC_ADC1_CLK_ENABLE();
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return false;
    }

    return HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
}

static bool ConfigureI2c(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    __HAL_RCC_I2C1_CLK_ENABLE();
    return HAL_I2C_Init(&hi2c1) == HAL_OK;
}

static bool ConfigureUart(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    __HAL_RCC_USART1_CLK_ENABLE();
    return HAL_UART_Init(&huart1) == HAL_OK;
}

static bool ConfigurePwmTimers(void)
{
    TIM_OC_InitTypeDef pwm = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 399U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM1_CLK_ENABLE();
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        return false;
    }

    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    return HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_4) == HAL_OK;
}

bool Board_Init(void)
{
    HAL_RCC_DeInit();
    if (!ConfigureClock())
    {
        return false;
    }

    ConfigureGpio();
    if (!ConfigureAdc() ||
        !ConfigureI2c() ||
        !ConfigureUart() ||
        !ConfigurePwmTimers())
    {
        return false;
    }

    if (!MPU6050_Init(&(MPU6050_Config){
            .i2c = &hi2c1,
            .address = MPU6050_I2C_ADDRESS
        }) ||
        !Bluetooth_Init(&huart1))
    {
        return false;
    }

    return true;
}

const Sensor_Config *Board_GetSensorConfig(void)
{
    return &sensor_config;
}

const Motor_Config *Board_GetMotorConfig(void)
{
    return &motor_config;
}
