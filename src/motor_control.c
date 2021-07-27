#include "motor_control.h"
#include "gpio.h"
#include <stdbool.h>

#include "gd32vf103_timer.h"

#define MOTOR_COUNT 4

/**
 * @brief 电机配置
 * 
 */
struct Motor motors[MOTOR_COUNT] = {
    { .PinCtrlA = GPIO_PA(4),
        .PinCtrlB = GPIO_PA(5),
        .PWMChannel = 0,
        .OdomChannel = 0 },
    { .PinCtrlA = GPIO_PA(6),
        .PinCtrlB = GPIO_PA(7),
        .PWMChannel = 1,
        .OdomChannel = 1 },
    { .PinCtrlA = GPIO_PB(12),
        .PinCtrlB = GPIO_PB(13),
        .PWMChannel = 2,
        .OdomChannel = 2 },
    { .PinCtrlA = GPIO_PB(14),
        .PinCtrlB = GPIO_PB(15),
        .PWMChannel = 3,
        .OdomChannel = 3 },
};

void motor_SetPWM(struct Motor* motor, bool cw, uint16_t pwm)
{
    // 设置方向
    gpio_bit_write(GPIO_PIN(motor->PinCtrlA), cw);
    gpio_bit_write(GPIO_PIN(motor->PinCtrlB), !cw);

    // 设置占空比
    timer_channel_output_pulse_value_config(TIMER0, motor->PWMChannel, pwm);
}

/**
 * @brief 初始化电机
 * 
 */
void motor_InitMotor()
{
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_deinit(TIMER0);
    
    // 初始化Timer
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 5399;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 10000; // 周期
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE; // 禁用对偶输出
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;

    // GPIO 重映射，将外设资源给PWM输出
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        struct Motor motor = motors[i];
        // IO 控制初始化
        gpio_init_pin(motor.PinCtrlA, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ);
        gpio_init_pin(motor.PinCtrlB, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ);

        timer_channel_output_config(TIMER0, motor.PWMChannel, &timer_ocinitpara);
        timer_channel_output_mode_config(TIMER0, motor.PWMChannel, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(TIMER0, motor.PWMChannel, TIMER_OC_SHADOW_DISABLE);
    }

    timer_primary_output_config(TIMER0, ENABLE);
    timer_auto_reload_shadow_enable(TIMER0);
    timer_enable(TIMER0);
}

void motor_InitOdom() {
    
}