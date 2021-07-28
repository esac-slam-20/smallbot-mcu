/**
 * @file gpio.h
 * @author Jim Jiang (jim@lotlab.org)
 * @brief 简单的GPIO操作封装
 * @version 0.1
 * @date 2021-07-28
 * 
 * @copyright Copyright (c) GDUT ESAC 2021
 * 
 */

#pragma once
#include "gd32vf103_gpio.h"

#define _GPIO_BANK(pin) (GPIO_BASE + (pin >> 5) * 0x400)
#define _GPIO_PIN(pin) (1 << (pin & 0x1F))

// 统一化PIN
#define GPIO_PIN(pin) _GPIO_BANK(pin), _GPIO_PIN(pin)

#define GPIO_PA(pin) (0x00 + pin)
#define GPIO_PB(pin) (0x20 + pin)
#define GPIO_PC(pin) (0x40 + pin)
#define GPIO_PD(pin) (0x60 + pin)
#define GPIO_PE(pin) (0x80 + pin)

#define gpio_init_pin(pin, mode, speed) gpio_init(_GPIO_BANK(pin), mode, speed, _GPIO_PIN(pin))

