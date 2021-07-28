#include "gd32vf103_gpio.h"
#include "systick.h"
#include <stdint.h>

#include "motor_control.h"

static void init()
{
    // 禁用SWJ接口
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE);

    // todo: 初始化UART2，PB10, PB11

    // 初始化控制
    motor_Init();

    // 初始化通信接口
    
}

int main()
{
    init();

    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_13);

    while (1) {
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(500);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(500);
    }
}
