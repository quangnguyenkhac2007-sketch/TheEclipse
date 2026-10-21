#ifndef BOARD_H
#define BOARD_H

#include "motor.h"
#include "sensor.h"

#include <stdbool.h>

bool Board_Init(void);
const Sensor_Config *Board_GetSensorConfig(void);
const Motor_Config *Board_GetMotorConfig(void);

#endif /* BOARD_H */
