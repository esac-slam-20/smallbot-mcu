#pragma once

enum CommCmd {
    // 参数相关
    CMD_ACK = 0x00, // ACK
    CMD_NACK, // NACK
    CMD_INVALID_ARG, // 参数错误

    CMD_SET_SPEED = 0x10, // 设置速度
    
    CMD_PARAM_ENCODER = 0x20, // 配置编码器
    CMD_PARAM_PID = 0x21, // 配置PID参数
    CMD_PARAM_SAVE = 0x2E, // 保存配置参数
    CMD_PARAM_IGNORE = 0x2F, // 忽略配置参数
    
    CMD_INFO_ODOM = 0x80, // 里程计信息
};
