#ifndef CURRENT_CTRL_H
#define CURRENT_CTRL_H

#include "motor_config.h"

extern float current[MOTOR_MAX];
extern float current_max[MOTOR_MAX];

void Current_Command(int i, float value);

#endif
