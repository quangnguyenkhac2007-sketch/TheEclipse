#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;
} MPU6050_Config;

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temperature_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} MPU6050_Data;

bool MPU6050_Init(const MPU6050_Config *config);
bool MPU6050_Read(MPU6050_Data *data);

#endif /* MPU6050_H */
