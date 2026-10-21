#ifndef CONFIG_H
#define CONFIG_H

/* Tune these values after the sensor and motor hardware is configured. */
#define LINE_FOLLOWER_CONTROL_PERIOD_S 0.010f
#define LINE_FOLLOWER_BASE_SPEED       0.0f

#define LINE_FOLLOWER_PID_KP           0.0f
#define LINE_FOLLOWER_PID_KI           0.0f
#define LINE_FOLLOWER_PID_KD           0.0f
#define LINE_FOLLOWER_PID_OUTPUT_MIN  (-100.0f)
#define LINE_FOLLOWER_PID_OUTPUT_MAX   100.0f

#define LINE_SENSOR_COUNT               5U
#define LINE_SENSOR_ADC_MAX          4095U
#define MPU6050_I2C_ADDRESS         (0x68U << 1U)

#endif /* CONFIG_H */