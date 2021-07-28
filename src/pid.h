#pragma once

#include <stdint.h>

/**
 * @brief PID控制电机
 * 
 * @param motor 电机编号
 * @param targetSpd 目标速度，rps
 * @param currentSpd 当前速度, rps
 * @return float 
 */
float pid_DoPID(uint8_t motor, float targetSpd, float currentSpd);
