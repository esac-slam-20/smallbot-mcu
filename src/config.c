/**
 * @file config.c
 * @author Jim Jiang (jim@lotlab.org)
 * @brief 配置项目相关，定义了配置的存储与读取。
 * @version 0.1
 * @date 2021-07-28
 * 
 * @copyright Copyright (c) GDUT ESAC 2021
 * 
 */
#include "config.h"
#include <stdint.h>

#include <string.h>

// PID 配置
struct PIDParam config_PIDParam = { 1.0, 0.1, 0.1 };
// 编码器数值
uint16_t config_EncoderTicks = 333;

void config_Read()
{
    // todo: read from EEPROM
}

void config_Write()
{
    // todo: write to EEPROM
}

void config_Init()
{
    config_Read();
}

void config_SetPIDParam(struct PIDParam* param)
{
    memcpy(&config_PIDParam, param, sizeof(config_PIDParam));
}

void config_SetEncoderTicks(uint16_t ticks)
{
    config_EncoderTicks = ticks;
}
