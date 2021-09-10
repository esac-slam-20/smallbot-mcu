#include "batt.h"
#include "gd32vf103_adc.h"
#include "gd32vf103_gpio.h"

#include "communication.h"

/**
 * @brief 初始化电量测量
 * 
 */
void batt_Init()
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);

    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

    adc_deinit(ADC1);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1U);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    /* ADC external trigger config */
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC1);
    delay_1ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
}

static volatile uint16_t voltage = 12000;

/**
 * @brief 电量测量
 * 
 */
void batt_Measure()
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0U, ADC_CHANNEL_8, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while (!adc_flag_get(ADC1, ADC_FLAG_EOC))
        ;
    /* clear the end of conversion flag */
    adc_flag_clear(ADC1, ADC_FLAG_EOC);
    /* return regular channel sample value */
    uint16_t raw = adc_regular_data_read(ADC1);

    voltage = (uint32_t)raw * 3300 / 4096;
}

/**
 * @brief 发送电压值
 * 
 */
void batt_SendVoltage()
{
    comm_SendBatt(voltage);
}
