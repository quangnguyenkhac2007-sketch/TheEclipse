#include "mpu6050.h"

#include "config.h"

#include <stddef.h>

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_DATA_LENGTH      14U

static I2C_HandleTypeDef *mpu_i2c;
static uint16_t mpu_address;

static bool WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(mpu_i2c,
                             mpu_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             100U) == HAL_OK;
}

static int16_t ReadSignedWord(const uint8_t *data)
{
    uint16_t value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return (int16_t)value;
}

bool MPU6050_Init(const MPU6050_Config *config)
{
    if ((config == NULL) || (config->i2c == NULL) ||
        ((config->address & 0x01U) != 0U))
    {
        return false;
    }

    mpu_i2c = config->i2c;
    mpu_address = (config->address == 0U)
                      ? (uint16_t)MPU6050_I2C_ADDRESS
                      : config->address;

    if (HAL_I2C_IsDeviceReady(mpu_i2c, mpu_address, 3U, 100U) != HAL_OK)
    {
        return false;
    }

    HAL_Delay(100U);
    return WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x00U) &&
           WriteRegister(MPU6050_REG_SMPLRT_DIV, 0x07U) &&
           WriteRegister(MPU6050_REG_CONFIG, 0x03U) &&
           WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x00U) &&
           WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00U);
}

bool MPU6050_Read(MPU6050_Data *data)
{
    uint8_t buffer[MPU6050_DATA_LENGTH];

    if ((data == NULL) || (mpu_i2c == NULL))
    {
        return false;
    }

    if (HAL_I2C_Mem_Read(mpu_i2c,
                         mpu_address,
                         MPU6050_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         MPU6050_DATA_LENGTH,
                         100U) != HAL_OK)
    {
        return false;
    }

    data->accel_x_raw = ReadSignedWord(&buffer[0]);
    data->accel_y_raw = ReadSignedWord(&buffer[2]);
    data->accel_z_raw = ReadSignedWord(&buffer[4]);
    data->temperature_raw = ReadSignedWord(&buffer[6]);
    data->gyro_x_raw = ReadSignedWord(&buffer[8]);
    data->gyro_y_raw = ReadSignedWord(&buffer[10]);
    data->gyro_z_raw = ReadSignedWord(&buffer[12]);

    data->accel_x_g = (float)data->accel_x_raw / 16384.0f;
    data->accel_y_g = (float)data->accel_y_raw / 16384.0f;
    data->accel_z_g = (float)data->accel_z_raw / 16384.0f;
    data->temperature_c = ((float)data->temperature_raw / 340.0f) + 36.53f;
    data->gyro_x_dps = (float)data->gyro_x_raw / 131.0f;
    data->gyro_y_dps = (float)data->gyro_y_raw / 131.0f;
    data->gyro_z_dps = (float)data->gyro_z_raw / 131.0f;
    return true;
}
