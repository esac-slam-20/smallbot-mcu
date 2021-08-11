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

#include "debug.h"
#include <string.h>

// PID 配置
struct PIDParam config_PIDParam = { 0.055, 819, 0 };
// 编码器数值
uint16_t config_EncoderTicks = 333;

void config_Read()
{
    // 草，没有dataflash给我读
}

void config_Write()
{
    // 草，没有dataflash给我写
}

void config_Init()
{
    config_Read();
}

void config_SetPIDParam(struct PIDParam* param)
{
    PRINT("Set PID param to P: %d, I: %d, D: %d.\r\n", (int)param->Prop, (int)param->Int, (int)param->Diff);
    config_PIDParam.Prop = param->Prop;
    config_PIDParam.Int = param->Int;
    config_PIDParam.Diff = param->Diff;
}

void config_SetEncoderTicks(uint16_t ticks)
{
    PRINT("Set encoder ticks to %d.\r\n", ticks);
    config_EncoderTicks = ticks;
}
