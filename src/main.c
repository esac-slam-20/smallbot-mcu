#include "gd32vf103_gpio.h"
#include "systick.h"
#include <stdint.h>

int main()
{
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_13);

    while (1) {
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(500);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(500);
    }
}
