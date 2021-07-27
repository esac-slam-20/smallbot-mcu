#pragma once

#include <stdint.h>

/**
 * @brief PID参数
 * 
 */
struct PIDParam
{
    float Prop; // 比例
    float Int; // 积分
    float Diff; // 微分
};

/**
 * @brief PID配置
 * 
 */
extern struct PIDParam config_PIDParam;

/**
 * @brief 编码器配置，电机一圈有多少个Tick
 * 
 */
extern uint16_t config_EncoderTicks;

/**
 * @brief 从存储器中读取配置
 * 
 */
void config_Read();

/**
 * @brief 写当前配置到存储器
 * 
 */
void config_Write();

/**
 * @brief 初始化配置服务
 * 
 */
void config_Init();

/**
 * @brief 设置PID的参数
 * 
 * @param params [Pro, Int, Dif]
 */
void config_SetPIDParam(struct PIDParam* param);

/**
 * @brief 设置电机转一圈编码器的脉冲数目
 * 
 * @param ticks 
 */
void config_SetEncoderTicks(uint16_t ticks);
