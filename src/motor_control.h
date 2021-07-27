#pragma once

#include <stdint.h>

/**
 * @brief 电机配置
 * 
 */
struct Motor
{
    // 控制脚A
    uint8_t PinCtrlA;
    // 控制脚B
    uint8_t PinCtrlB;
    // PWM通道
    uint8_t PWMChannel;
    // 里程计的通道
    uint8_t OdomChannel;
};
