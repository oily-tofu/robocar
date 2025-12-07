#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct
{
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数

    float Ek;        // 当前误差
    float Ek_1;      // 上一次误差
    float Ek_2;      // 上上次误差

    float Output;    // 当前输出
    float Out_Min;   // 输出最小值
    float Out_Max;   // 输出最大值
} PID_TypeDef;

/* 增量式 PID 初始化 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

/* 增量式计算（返回增量 Δu） */
float PID_Calc_Incremental(PID_TypeDef *pid, float target, float feedback);

#endif
