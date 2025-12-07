#include "pid.h"

/**
 * @brief   增量式 PID 初始化
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->Ek   = 0;
    pid->Ek_1 = 0;
    pid->Ek_2 = 0;

    pid->Output = 0;

    pid->Out_Min = out_min;
    pid->Out_Max = out_max;
}

/**
 * @brief   增量式 PID 计算
 * @param   target   给定值 (期望速度)
 * @param   feedback 实际值 (当前速度)
 * @return  Δu（增量量）
 */
float PID_Calc_Incremental(PID_TypeDef *pid, float target, float feedback)
{
    /* 计算误差 */
    pid->Ek = target - feedback;

    /* 增量式 PID 公式 */
    float delta_u = 
          pid->Kp * (pid->Ek - pid->Ek_1)
        + pid->Ki * pid->Ek
        + pid->Kd * (pid->Ek - 2 * pid->Ek_1 + pid->Ek_2);

    /* 更新输出（u(k) = u(k-1) + Δu) */
    pid->Output += delta_u;

    /* 限幅 */
    if (pid->Output > pid->Out_Max) pid->Output = pid->Out_Max;
    if (pid->Output < pid->Out_Min) pid->Output = pid->Out_Min;

    /* 更新误差值 */
    pid->Ek_2 = pid->Ek_1;
    pid->Ek_1 = pid->Ek;

    return pid->Output;
}
