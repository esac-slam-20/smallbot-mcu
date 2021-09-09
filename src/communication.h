#pragma once

#include <stdint.h>

enum CommCmd {
    // 参数相关
    CMD_ACK = 0x00, // ACK
    CMD_NACK, // NACK
    CMD_INVALID_ARG, // 参数错误

    CMD_SET_SPEED = 0x10, // 设置速度
    CMD_GET_ODOM = 0x11, // 获取ODOM
    CMD_GET_BATT = 0x12, // 获取电池电量信息
    
    CMD_PARAM_ENCODER = 0x20, // 配置编码器
    CMD_PARAM_PID = 0x21, // 配置PID参数
    CMD_PARAM_SAVE = 0x2E, // 保存配置参数
    CMD_PARAM_IGNORE = 0x2F, // 忽略配置参数
    
    CMD_INFO_ODOM = 0x80, // 里程计信息
    CMD_INFO_BATT = 0x81, // 电池电量信息
};

/**
 * @brief 发送里程计信息
 * 
 * @param odoms 4个里程计的数值
 */
void comm_SendOdom(int32_t odoms[]);

/**
 * @brief 发送电池电压数据
 * 
 * @param mv 电压数据，mv
 */
void comm_SendBatt(uint16_t mv);

/**
 * @brief 初始化通信串口
 * 
 */
void comm_Init();
