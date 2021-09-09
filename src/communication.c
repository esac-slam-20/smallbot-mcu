/**
 * @file communication.c
 * @author Jim Jiang (jim@lotlab.org)
 * @brief 通讯相关。定义了通讯协议，并处理底层串口收发
 * @version 0.1
 * @date 2021-07-28
 * 
 * @copyright Copyright (c) GDUT ESAC 2021
 * 
 */

#include "communication.h"
#include "config.h"
#include "motor_control.h"

#include "gd32vf103_dma.h"
#include "gd32vf103_eclic.h"
#include "gd32vf103_usart.h"
#include "gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ARRAYNUM(arr_nanme) (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))

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

#define MAX_DATA_SIZE 32

// 数据暂存
static uint8_t data_buffer[MAX_DATA_SIZE] = { 0 };
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
    // todo: 判断CRC
    return true;
}

#define RING_BUFFER(name, type, size)      \
    const uint16_t name##_size = size;     \
    static type name##_buff[size] = { 0 }; \
    static volatile uint16_t name##_head = 0, name##_tail = 0;

// 串口发送Ringbuffer
RING_BUFFER(uart_tx, uint8_t, 64);

static void trySend()
{
    if (RESET == usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)) {
        usart_data_transmit(USART1, uart_tx_buff[uart_tx_tail++]);
        uart_tx_tail %= uart_tx_size;

        usart_interrupt_enable(USART1, USART_INT_TBE);
    }
}

/**
 * @brief 串口发送数据
 * 
 * @param dat 
 * @param len 
 */
static void comm_Tx(uint8_t* dat, uint8_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_tx_buff[uart_tx_head++] = dat[i];
        uart_tx_head %= uart_tx_size;
    }

    // 当前没有正在发送的指令,直接发送.
    trySend();
}

/**
 * @brief 发送里程计信息
 * 
 * @param odoms 4个里程计的数值
 */
void comm_SendOdom(int32_t* odoms)
{
    uint8_t dat[] = {
        MAGIC_NUM_HEAD,
        CMD_INFO_ODOM, // Cmd
        16, // Len
        0x00, 0x00, 0x00, 0x00, // Data
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, // Checksum
        MAGIC_NUM_END
    };

    for (size_t i = 0; i < 12; i++) {
        dat[3 + i] = ((uint8_t*)odoms)[i];
    }

    comm_Tx(dat, sizeof(dat));
}

/**
 * @brief 发送电池电压数据
 * 
 * @param mv 电压数据，mv
 */
void comm_SendBatt(uint16_t mv)
{
    uint8_t dat[] = {
        MAGIC_NUM_HEAD,
        CMD_INFO_BATT, // Cmd
        2, // Len
        0x00, 0x00, // Data
        0x00, 0x00, // Checksum
        MAGIC_NUM_END
    };

    dat[3] = (mv >> 0) & 0x100;
    dat[4] = (mv >> 8) & 0x100;

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
    // 检查校验和
    if (!comm_ChecksumValidate()) {
        return comm_AckState(CMD_NACK);
    }

    switch (data_cmd) {
    case CMD_SET_SPEED: // 设置电机目标速度
        if (data_len != 8)
            return comm_AckState(CMD_INVALID_ARG);
        motor_SetSpeed((int16_t*)data_buffer);
        comm_AckState(CMD_ACK);
        break;
    case CMD_GET_ODOM:
        if (data_len != 0)
            return comm_AckState(CMD_INVALID_ARG);
        motor_SendOdom();
        break;
    case CMD_PARAM_ENCODER: // 配置编码器
        if (data_len != 2)
            return comm_AckState(CMD_INVALID_ARG);
        config_SetEncoderTicks(*(uint16_t*)data_buffer);
        comm_AckState(CMD_ACK);
        break;
    case CMD_PARAM_PID: // 配置PID
        if (data_len != 12)
            return comm_AckState(CMD_INVALID_ARG);
        config_SetPIDParam((struct PIDParam*)data_buffer);
        comm_AckState(CMD_ACK);
        break;
    case CMD_PARAM_SAVE: // 保存参数
        if (data_len != 0)
            return comm_AckState(CMD_INVALID_ARG);
        config_Write();
        comm_AckState(CMD_ACK);
        break;
    case CMD_PARAM_IGNORE: // 读取参数
        if (data_len != 0)
            return comm_AckState(CMD_INVALID_ARG);
        config_Read();
        comm_AckState(CMD_ACK);
        break;
    default:
        return comm_AckState(CMD_INVALID_ARG);
    }
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
            checksum_index = 0;
        }
        break;
    case STAGE_DAT:
        data_buffer[data_index++] = dat;
        if (data_index >= data_len || data_index >= MAX_DATA_SIZE) {
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
            // 错误结束符号，直接重置状态忽略数据
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 初始化通信串口
 * 
 */
void comm_Init()
{
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_GPIOA);

    // 初始化 PIN
    gpio_init_pin(GPIO_PA(2), GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PA(3), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);

    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);

    eclic_irq_enable(USART1_IRQn, 1, 1);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
}

void USART1_IRQHandler(void)
{
    // 数据接收中断
    if (SET == usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) {
        uint8_t dat = usart_data_receive(USART1);
        comm_Rx(dat);
    }
    // 数据发送中断
    if (SET == usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)) {
        usart_interrupt_flag_clear(USART1, USART_INT_FLAG_TBE);
        if (uart_tx_head != uart_tx_tail) {
            usart_data_transmit(USART1, uart_tx_buff[uart_tx_tail++]);
            uart_tx_tail %= uart_tx_size;
        } else {
            usart_interrupt_disable(USART1, USART_INT_TBE);
        }
    }
}
