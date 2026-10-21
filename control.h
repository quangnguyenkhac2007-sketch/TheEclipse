#ifndef CONTROL_H
#define CONTROL_H

#include "motor.h"
#include "sensor.h"

#include <stdbool.h>

bool Control_Init(const Sensor_Config *sensor_config,
                  const Motor_Config *motor_config);
bool Control_Update(float dt_seconds);
void Control_Stop(void);

#endif /* CONTROL_H */