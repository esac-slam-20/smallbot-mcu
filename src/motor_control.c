/**
 * @file motor_control.c
 * @author Jim Jiang (jim@lotlab.org)
 * @brief 电机控制相关代码。负责接受速度，控制PWM，并读取编码器信息。
 * @version 0.1
 * @date 2021-07-28
 * 
 * @copyright Copyright (c) GDUT ESAC 2021
 * 
 */

#include "motor_control.h"
#include "communication.h"
#include "config.h"
#include "gpio.h"
#include "pid.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "gd32vf103_eclic.h"
#include "gd32vf103_timer.h"

// 电机数目
#define MOTOR_COUNT 4
// PWM 分辨率
#define PWM_MAX 1000
// 电机最低控制速度，rps。小于此速度时直接锁电机。
#define MIN_PID_SPEED 1

#define ABS(a) ((a) > 0 ? (a) : -(a))

/**
 * @brief 电机配置
 * 
 */
struct Motor motors[MOTOR_COUNT] = {
    { .PinCtrlA = GPIO_PA(4),
        .PinCtrlB = GPIO_PA(5),
        .PWMChannel = 0,
        .OdomChannel = 1 },
    { .PinCtrlA = GPIO_PA(6),
        .PinCtrlB = GPIO_PA(7),
        .PWMChannel = 1,
        .OdomChannel = 2 },
    { .PinCtrlA = GPIO_PB(12),
        .PinCtrlB = GPIO_PB(13),
        .PWMChannel = 2,
        .OdomChannel = 3 },
    { .PinCtrlA = GPIO_PB(14),
        .PinCtrlB = GPIO_PB(15),
        .PWMChannel = 3,
        .OdomChannel = 0 },
};

/**
 * @brief 设置指定电机的PWM值
 * 
 * @param motor 电机
 * @param cw 正反转
 * @param pwm PWM值
 */
static void motor_setPWM(struct Motor* motor, bool cw, uint16_t pwm)
{
    if (pwm > 0) {
        // 设置方向
        // 右手定则，轮子向电机看
        gpio_bit_write(GPIO_PIN(motor->PinCtrlA), !cw);
        gpio_bit_write(GPIO_PIN(motor->PinCtrlB), cw);
    } else {
        // 锁电机
        gpio_bit_write(GPIO_PIN(motor->PinCtrlA), false);
        gpio_bit_write(GPIO_PIN(motor->PinCtrlB), false);
    }
    // 设置占空比
    timer_channel_output_pulse_value_config(TIMER0, motor->PWMChannel, pwm);
}

/**
 * @brief 设置指定电机的PWM值
 * 
 * @param motor 电机序号
 * @param pwm PWM值，范围为[-1, 1]。正数正转，负数反转。
 */
static void motor_SetPWM(uint8_t motor, float pwm)
{
    if (motor >= MOTOR_COUNT)
        return;
    motor_setPWM(&motors[motor], pwm > 0, ABS(pwm) * PWM_MAX);
}

/**
 * @brief 初始化电机
 * 
 */
static void motor_InitMotor()
{
    timer_parameter_struct timer_initpara = {
        .prescaler = 9, // 分频系数
        .alignedmode = TIMER_COUNTER_EDGE,
        .clockdivision = TIMER_CKDIV_DIV1,
        .counterdirection = TIMER_COUNTER_UP,
        .period = PWM_MAX, // PWM计数MAX
        .repetitioncounter = 0,
    };

    timer_oc_parameter_struct timer_ocinitpara = {
        .outputstate = TIMER_CCX_ENABLE, // 启用输出
        .outputnstate = TIMER_CCXN_DISABLE, // 禁用对偶输出
        .ocpolarity = TIMER_OC_POLARITY_HIGH, // 高极性
        .ocidlestate = TIMER_OC_IDLE_STATE_LOW, // 默认低电平
    };

    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOA);

    timer_deinit(TIMER0);

    // 初始化Timer
    timer_init(TIMER0, &timer_initpara);

    // GPIO 初始化，设定为AF
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        struct Motor* motor = &motors[i];
        // IO 控制初始化
        gpio_init_pin(motor->PinCtrlA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ);
        gpio_init_pin(motor->PinCtrlB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ);
        gpio_bit_reset(GPIO_PIN(motor->PinCtrlA));
        gpio_bit_reset(GPIO_PIN(motor->PinCtrlB));

        // PWM输出配置
        timer_channel_output_config(TIMER0, motor->PWMChannel, &timer_ocinitpara);
        timer_channel_output_mode_config(TIMER0, motor->PWMChannel, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(TIMER0, motor->PWMChannel, TIMER_OC_SHADOW_DISABLE);
    }

    timer_primary_output_config(TIMER0, ENABLE);
    timer_auto_reload_shadow_enable(TIMER0);
    timer_enable(TIMER0);
}

/**
 * @brief 初始化光电编码器
 * 
 */
static void motor_InitEncoder()
{
    timer_ic_parameter_struct timer_icinitpara = {
        .icpolarity = TIMER_IC_POLARITY_RISING,
        .icselection = TIMER_IC_SELECTION_DIRECTTI,
        .icprescaler = TIMER_IC_PSC_DIV1,
        .icfilter = 0
    };

    timer_parameter_struct timer_initpara = {
        .prescaler = 0, // 分频系数
        .alignedmode = TIMER_COUNTER_EDGE,
        .counterdirection = TIMER_COUNTER_UP,
        .period = 65535, // 最大周期
        .clockdivision = TIMER_CKDIV_DIV1,
        .repetitioncounter = 0
    };

    // 重映射 GPIO
    // TIMER1:
    gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP0, ENABLE); // PA15, PB3
    // TIMER2:
    gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE); // PB4, PB5
    // TIMER3: PB6, PB7
    // TIMER4: A0, A1

    // GPIO初始化
    gpio_init_pin(GPIO_PA(15), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(3), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(4), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(5), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(6), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PB(7), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PA(0), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);
    gpio_init_pin(GPIO_PA(1), GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ);

    // 启用外设时钟
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_TIMER4);

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        uint32_t timer = TIMER1 + motors[i].OdomChannel * 0x400;

        // 初始化外设
        timer_deinit(timer);
        timer_init(timer, &timer_initpara);

        // 输入捕获模式
        timer_input_capture_config(timer, TIMER_CH_0, &timer_icinitpara);
        timer_input_capture_config(timer, TIMER_CH_1, &timer_icinitpara);
        timer_quadrature_decoder_mode_config(timer, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_BOTH_EDGE, TIMER_IC_POLARITY_BOTH_EDGE);

        // 启用 Timer
        timer_auto_reload_shadow_enable(timer);
        timer_enable(timer);
    }
}

/**
 * @brief 初始化主定时器
 * 
 */
static void motor_InitTimer()
{
    timer_parameter_struct timer_initpara = {
        .prescaler = 10799,
        .alignedmode = TIMER_COUNTER_EDGE,
        .counterdirection = TIMER_COUNTER_UP,
        .period = 50,
        .clockdivision = TIMER_CKDIV_DIV1,
    };

    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER5);
    timer_init(TIMER5, &timer_initpara);

    eclic_irq_enable(TIMER5_IRQn, 0, 0);
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    timer_enable(TIMER5);
}

/**
 * @brief 初始化所有电机相关外设
 * 
 */
void motor_Init()
{
    motor_InitMotor();
    motor_InitEncoder();
    motor_InitTimer();
}

/**
 * @brief 电机目标速度，rpm
 * 
 */
static int16_t motor_targetSpeeds[4];

#define COUNTER_RELOAD 200 // 1s

/**
 * @brief 数据存活计数器
 * 
 */
static uint16_t data_valid_counter = 0;

/**
 * @brief 设置电机目标速度
 * 
 * @param speeds 电机速度，rpm
 */
void motor_SetSpeed(int16_t speeds[4])
{
    data_valid_counter = COUNTER_RELOAD;
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        motor_targetSpeeds[i] = speeds[i];
    }
    printf("Set motor speed to %d %d %d %d.\r\n", speeds[0], speeds[1], speeds[2], speeds[3]);
}

/**
 * @brief 32位编码器值
 * 
 */
static int32_t decoder_val[4] = { 0 };

/**
 * @brief 16位编码器值
 * 
 */
static uint16_t decoder_last_val[4] = { 0 };

/**
 * @brief 电机控制过程，负责读取编码器并调用PID
 * 
 */
static void motor_Routine()
{
    // 读取编码器数据
    int32_t delta[4] = { 0 };
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        uint8_t index = motors[i].OdomChannel;
        uint32_t timer = TIMER1 + index * 0x400;

        uint32_t v = timer_counter_read(timer);
        delta[i] = v - decoder_last_val[i];
        delta[i] = -delta[i]; // 反转delta

        // 修正Overflow
        if (ABS(delta[i]) > 0x8000) {
            delta[i] -= delta[i] > 0 ? 0x10000 : -0x10000;
        }
        // 1信号=4Tick
        decoder_val[i] += delta[i] / 4;

        decoder_last_val[i] = v;
    }

    // 防止上层控制失效导致速度一直保持
    if (data_valid_counter > 0) {
        data_valid_counter--;
        if (data_valid_counter == 0) {
            motor_targetSpeeds[0] = 0;
            motor_targetSpeeds[1] = 0;
            motor_targetSpeeds[2] = 0;
            motor_targetSpeeds[3] = 0;
        }
    }

    // PID 控制电机
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        float targetSpeed = motor_targetSpeeds[i] / 60.0f;
        float currentSpeed = delta[i] / 4 / 0.005f / config_EncoderTicks;
        float val = 0;
        // 判断是否可以PID
        if (ABS(targetSpeed) > MIN_PID_SPEED) {
            val = pid_DoPID(i, targetSpeed, currentSpeed);
        }
        motor_SetPWM(i, val);
    }

    // 上报数据
    comm_SendOdom(decoder_val);
}

/**
 * @brief 中断处理
 * 
 */
void TIMER5_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_UP)) {
        timer_interrupt_flag_clear(TIMER5, TIMER_INT_UP);
        // 执行周期操作
        motor_Routine();
    }
}