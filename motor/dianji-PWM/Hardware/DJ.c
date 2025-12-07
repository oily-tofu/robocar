#include "stm32f10x.h"
#include "DJ.h"
#include "oled.h"
#include <string.h>
#include <stdio.h> 
#include "serial.h"
////////////////////////////////////////////////////////////////////////
// #include "stm32f10x.h"
// #include "Delay.h"
// #include "DJ.h"
// #include "serial.h"
// #include <string.h>


// int16_t speed;
// char buf[16];

// int main()
// {
//     Serial_Init();
//     Encoder_Init_TIM3();

//     while(1)
//     {
//         speed = Encoder_GetSpeed();

//         sprintf(buf, "%d\r\n", speed);
//         DMA_Send((uint8_t*)buf, strlen(buf));

//         Delay_ms(10);
//     }
// }
/////////////////////////////////////////////////////////////////////////



void DJ_Init(void)
{
    /* 1. 定义初始化结构体 */
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    /* 2. 开启外设时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    
    /* 3. 配置引脚 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 4. 配置 PA5、PA6 为输入（上拉输入） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    
    /* 5. 设置默认输出电平 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}

/* 读取 PA5、PA6 电平状态函数 */
uint8_t Read_PA5(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
}

uint8_t Read_PA6(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// 编码器计数器初始化（TIM3，PA6/PA7）


// 全局变量 - 用于在中断和主程序之间传递数据
volatile int32_t encoder_speed_value = 0;    // 编码器速度值
volatile float speedM = 0;
volatile int32_t located = 0;
volatile uint8_t speed_update_flag = 0;      // 速度更新标志




void Encoder_Init_TIM3(void)
{
    /* ----------- 1. 开启时钟 ----------- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* ----------- 2. 初始化 PA6、PA7 为上拉输入 ----------- */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ----------- 3. 配置 TIM3 为编码器模式（X4 计数） ----------- */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;  // 最大计数
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_EncoderInterfaceConfig(TIM3,
                               TIM_EncoderMode_TI12,            // X4 模式
                               TIM_ICPolarity_Rising,           // A 上升沿
                               TIM_ICPolarity_Rising);          // B 上升沿

    /* 输入滤波器(防抖动) */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6; // 可调 0–15
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* ----------- 4. 清零计数器并启动 ----------- */
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

//获取编码器的计数值（位置）
int16_t Encoder_GetTIM3(void)
{
    return (int16_t)TIM3->CNT;
}

//实时清零计数（用于速度计算）
int32_t Encoder_GetSpeed(void)
{
    int32_t speed = (int32_t)TIM3->CNT;
    TIM3->CNT = 0;     // 清零，便于计算速度
    return speed;
}


//////////////////////////////////////////////////////////////////////////////////////////////


// TIM2 初始化，速度更新，OLED 显示
void TIM2_OLED_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2. 配置时基，得到大约 0.01 秒间隔（可根据需求调整）
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;       // ARR
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;    // PSC, 72MHz / 7200 = 10kHz 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 3. 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 4. 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

// TIM2中断服务函数
void TIM2_IRQHandler(void)
{  
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

        encoder_speed_value = Encoder_GetSpeed();          // 获取编码器速度并清零
        speed_update_flag = 1;                             // 设置速度更新标志
        
        speedM = (float)encoder_speed_value * 0.019f ;       //一圈0.251m,每0.01s采样一次，转换为m/s  //speedM = (float)speed_value / 44 / 0.01 / 30 * 2*3.24159*0.04 ;

        // located = Encoder_GetTIM3();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   // 清中断标志
    }
}


uint8_t Is_Speed_Updated(void)
{
    uint8_t flag;
    
    __disable_irq();
    flag = speed_update_flag;
    __enable_irq();
    
    return flag;
}




// 重置速度更新标志
uint16_t len;
char buf_1[20];
void Show_Speed(void)
{
    
    // char buf_2[20];
    //char buf_3[20];
    if (!Is_Speed_Updated()) {
        return;
    }

    speed_update_flag = 0;              // 清除标志

    sprintf(buf_1, "%+.3f\r\n", speedM);
    OLED_ShowString(0,0,(u8*)buf_1,16,1);

    // sprintf(buf_2, "%+.d\r\n", encoder_speed_value);
    // OLED_ShowString(0,16,(u8*)buf_2,16,1);

    // sprintf(buf_3, "%+.d\r\n", located);
    // OLED_ShowString(80,0,(u8*)buf_3,16,1);

    len = strlen(buf_1);
    if (len > 0)
    {
        USART2_DMA_Send((uint8_t*)buf_1, len);
    }

}


