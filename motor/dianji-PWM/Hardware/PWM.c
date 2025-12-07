#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "DJ.h"



/**
  * 函    数：PWM初始化（TIM4 CH1 → PB6）
  */
void PWM_Init(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // TIM4 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOB 时钟

    /* GPIO 初始化：PB6 = TIM4_CH1 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 使用内部时钟 */
    TIM_InternalClockConfig(TIM4);

    /* 时基单元配置 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 900 - 1;       // ARR  10khz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 8 - 1;    // PSC  
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /* 输出比较单元配置（PWM1 模式）*/
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比=0
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    /* TIM4 使能 */
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：PWM 设置比较值（通道 1）
  * CCR1 范围：0~200（由 ARR 决定）
  */
void PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM4, Compare);
}


////////////////////////////////////////////////////////////////////////////////////////////////////


/**
  * 函    数：TIM5 更新中断初始化（用于定时 PID 计算）
  */
void TIM5_Update_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_Prescaler = 3600 - 1; // PSC  
    t.TIM_Period = 100 - 1;     // ARR   
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &t);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef n;
    n.NVIC_IRQChannel = TIM5_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 1;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

    TIM_Cmd(TIM5, ENABLE);
}



extern PID_TypeDef speed_pid;
extern float target_speed;
extern volatile float speedM;
extern volatile uint8_t speed_update_flag;
volatile uint16_t pidout;
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        float encoder_val;
        float feedback, pid_output;

        // 读取编码器速度

        encoder_val = speedM;


        feedback = encoder_val;  // 转换成和目标单位一致

        // 计算增量式 PID 输出
        pid_output = PID_Calc_Incremental(&speed_pid, target_speed, feedback);
        pidout = (uint16_t)pid_output;
        PWM_SetCompare1((uint16_t)pid_output);

        // 清中断标志
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

