#include "pid.h"
#include "config.h"

static float Error = 0, Error_last = 0, Integral_Error = 0, Differential_Error = 0;
static float Proportion_OUT, Integral_OUT, Differential_OUT, PID_OUT;

float pid_DoPID(uint8_t motor, float targetSpd, float currentSpd)
{
    float Ki; // 积分
    float Kd; // 微分
    int T = 5; // 周期
    Ki = config_PIDParam.Prop * T * (1 / config_PIDParam.Int); //积分项系数，即提取出积分项公式中所有可人为设定的参数
    Kd = config_PIDParam.Prop * config_PIDParam.Diff * (1 / T); //微分项系数，即提取出微分项公式中所有可人为设定的参数

    Error = targetSpd - currentSpd; //偏差
    Integral_Error = Integral_Error + Error; //偏差的积分
    Differential_Error = Error - Error_last; //偏差的微分

    Proportion_OUT = config_PIDParam.Prop * Error; //比例项输出 = prop * 偏差
    Integral_OUT = Ki * Integral_Error; //积分项输出 = Ki * 偏差的积分
    Differential_OUT = Kd * Differential_Error; //微分项输出 = Kd * 偏差的微分

    PID_OUT = Proportion_OUT + Integral_OUT + Differential_OUT;
    //PID最终输出 = 比例项输出 + 积分项输出 + 微分项输出
    Error_last =Error;
    return PID_OUT;
}
