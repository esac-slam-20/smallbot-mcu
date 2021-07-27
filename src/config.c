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
