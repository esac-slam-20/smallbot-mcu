#include "gd32vf103_eclic.h"
#include "gd32vf103_usart.h"
#include "gpio.h"

#include "systick.h"
#include <stdint.h>

#include "communication.h"
#include "config.h"
#include "motor_control.h"

#include "stdio.h"
#include "debug.h"

/**
 * @brief 初始化串口调试用UART
 * 
 */
static void uart_init()
{
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_GPIOB);

    // 初始化 PIN
    gpio_init_pin(GPIO_PB(10), GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(11), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);

    /* USART configure */
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 57600);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
}

static void init()
{
    // 禁用SWJ接口
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE);

    uart_init(); // 初始化调试接口
    config_Init(); // 配置初始化
    motor_Init(); // 初始化电机控制
    comm_Init(); // 初始化通信接口
}

int main()
{
    _init();

    eclic_global_interrupt_enable();
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);

    init();

    PRINT("Hello, world.\r\n");

    /* enable the led clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);

    while (1) {
        delay_1ms(500);
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(500);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        PRINT(".\r\n");
    }
}

int _write(int fd, char* buf, int size)
{
    for (int i = 0; i < size; i++) {
        usart_data_transmit(USART2, buf[i]);
        while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET) {
        }
    }
    return size;
}
