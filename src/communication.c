#include "communication.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "config.h"

/**
 * @brief 通信解析状态机状态
 * 
 */
enum CommState {
    STAGE_HEAD, // 头部
    STAGE_CMD, // 命令
    STAGE_LEN, // 长度
    STAGE_DAT, // 数据
    STAGE_CHECKSUM, // 校验和
    STAGE_END // 尾部
};

// 命令头部魔术字
const uint8_t MAGIC_NUM_HEAD = 0x55;
// 命令尾部魔术字
const uint8_t MAGIC_NUM_END = 0xAA;

// 数据暂存
static uint8_t data_buffer[32] = { 0 };
// 校验和暂存
static uint8_t checksum[2] = { 0 }, checksum_index = 0;
// 命令暂存
static uint8_t data_cmd, data_index, data_len;

// 当前状态机状态
static enum CommState comm_state = STAGE_HEAD;

/**
 * @brief 判断校验和是否正确
 * 
 * @return true 
 * @return false 
 */
static bool comm_ChecksumValidate()
{
    return true;
}

/**
 * @brief 串口发送数据
 * 
 * @param dat 
 * @param len 
 */
static void comm_Tx(uint8_t* dat, uint8_t len)
{
    // todo:
}

/**
 * @brief 发送里程计信息
 * 
 * @param odoms 4个里程计的数值
 */
void comm_SendOdom(int32_t odoms[]) {
    uint8_t dat[] = { 
        MAGIC_NUM_HEAD,
        CMD_INFO_ODOM,
        12, 
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 
        MAGIC_NUM_END
    };

    memcpy(dat+3, odoms, 12);
    comm_Tx(dat, sizeof(dat));
}

/**
 * @brief 回应一个请求
 * 
 * @param state 
 */
static void comm_AckState(uint8_t state)
{
    uint8_t dat[] = { MAGIC_NUM_HEAD, state, 0, 0x00, 0x00, MAGIC_NUM_END };
    comm_Tx(dat, sizeof(dat));
}

/**
 * @brief 命令解析
 * 
 */
static void comm_CmdParser()
{
    if (!comm_ChecksumValidate()) {
        return comm_AckState(CMD_NACK);
    }

    switch (data_cmd) {
    case CMD_SET_SPEED: // 设置电机目标速度
        if (data_len != 8)
            return comm_AckState(CMD_INVALID_ARG);
        // todo: set speed
        break;
    case CMD_PARAM_ENCODER: // 配置编码器
        if (data_len != 2)
            return comm_AckState(CMD_INVALID_ARG);
        config_SetEncoderTicks(*(uint16_t*)data_buffer);
        break;
    case CMD_PARAM_PID: // 配置PID
        if (data_len != 12)
            return comm_AckState(CMD_INVALID_ARG);
        config_SetPIDParam((struct PIDParam *)data_buffer);
        break;
    case CMD_PARAM_SAVE:
        if (data_len != 0)
            return comm_AckState(CMD_INVALID_ARG);
        config_Write();
        break;
    case CMD_PARAM_IGNORE:
        if (data_len != 0)
            return comm_AckState(CMD_INVALID_ARG);
        config_Read();
        break;
    default:
        return comm_AckState(CMD_INVALID_ARG);
    }

    comm_AckState(CMD_ACK);
}

/**
 * @brief 串口数据处理过程
 * 
 * @param dat 
 */
void comm_Rx(uint8_t dat)
{
    switch (comm_state) {
    case STAGE_HEAD:
        if (dat == MAGIC_NUM_HEAD) {
            comm_state++;
        } else {
            // 数据错误，等待下次验证
        }
        break;
    case STAGE_CMD:
        data_cmd = dat;
        comm_state++;
        break;
    case STAGE_LEN:
        data_len = dat;
        data_index = 0;
        comm_state++;
        // LEN=0则跳过DAT
        if (data_len == 0) {
            comm_state++;
        }
        break;
    case STAGE_DAT:
        data_buffer[data_index++] = dat;
        if (data_index == data_len) {
            comm_state++;
            checksum_index = 0;
        }
        break;
    case STAGE_CHECKSUM:
        checksum[checksum_index++] = dat;
        if (checksum_index == 2) {
            comm_state++;
        }
        break;
    case STAGE_END:
        comm_state = STAGE_HEAD;
        if (dat == MAGIC_NUM_END) {
            comm_CmdParser();
        } else {
            // invalid end, skip and reset.
        }
        break;
    default:
        break;
    }
}