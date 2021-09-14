#include "pid.h"
#include "config.h"

struct MidVal {
    float Error_last;
    float Integral_Error;
};

static struct MidVal val[4] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
/**
 * @brief 
 * 
 * 
 * @param motor 电机ID
 * @param targetSpd 带方向电机目标速度单位rps
 * @param currentSpd 当前电机速度rps
 * @return float 
 */
float pid_DoPID(uint8_t motor, float targetSpd, float currentSpd)
{
    float maxVal = 1;
    float Ki; // 积分
    float Kd; // 微分
    float T = 5; // 周期
    float Error = 0.0;
    float Differential_Error = 0.0;
    float Proportion_OUT, Integral_OUT, Differential_OUT, PID_OUT;
    if (currentSpd * targetSpd < 0 ||currentSpd >167 || currentSpd <-167) {//避免反转跳变电机，以及currenSpd异常，先归零
        val[motor].Integral_Error = 0;
        val[motor].Error_last = 0;
        if(targetSpd<0)
            return -0.01;
        else if(targetSpd>0)
            return 0.01;
        return 0;
    }

    Ki = config_PIDParam.Prop * T * (1 / config_PIDParam.Int); //积分项系数，即提取出积分项公式中所有可人为设定的参数
    Kd = config_PIDParam.Prop * config_PIDParam.Diff * (1 / T); //微分项系数，即提取出微分项公式中所有可人为设定的参数

    Error = targetSpd - currentSpd; //偏差
    val[motor].Integral_Error = val[motor].Integral_Error + Error; //偏差的积分
    Differential_Error = Error - val[motor].Error_last; //偏差的微分

    Proportion_OUT = config_PIDParam.Prop * Error; //比例项输出 = prop * 偏差
    Integral_OUT = Ki * val[motor].Integral_Error; //积分项输出 = Ki * 偏差的积分
    Differential_OUT = Kd * Differential_Error; //微分项输出 = Kd * 偏差的微分

    PID_OUT = Proportion_OUT + Integral_OUT + Differential_OUT;
    if (PID_OUT < -maxVal)
        PID_OUT = -maxVal;
    if (PID_OUT > maxVal)
        PID_OUT = maxVal;
    //PID最终输出 = 比例项输出 + 积分项输出 + 微分项输出

    val[motor].Error_last = Error;
    return PID_OUT;
}
