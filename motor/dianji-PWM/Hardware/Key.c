#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "led.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);						//将PB1和PB11引脚初始化为上拉输入
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;		//定义变量，默认键码值为0
	
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
	{
		delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0);	//等待按键松手
		delay_ms(20);											//延时消抖
		KeyNum = 1;												//置键码为1
	}
	
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0)			//读PB11输入寄存器的状态，如果为0，则代表按键2按下
	{
		delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0);	//等待按键松手
		delay_ms(20);											//延时消抖
		KeyNum = 2;												//置键码为2
	}
	
	return KeyNum;			//返回键码值，如果没有按键按下，所有if都不成立，则键码为默认值0
}


void TIM6_Update_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_Prescaler = 3600 - 1;  // 分频
    t.TIM_Period = 100 - 1;      // 自动重装载
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &t);

    // 使能更新中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // NVIC 设置
    NVIC_InitTypeDef n;
    n.NVIC_IRQChannel = TIM6_IRQn;     // 非常关键！换成 TIM6_IRQn
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 3;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

    // 启动定时器
    TIM_Cmd(TIM6, ENABLE);
}

volatile uint8_t key_flag = 0;

void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        key_flag = 1;   // 设置标志
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}

void PIDset(void)
{
	if(key_flag == 1)
		{
		uint8_t Keynum = Key_GetNum();
		if(Keynum == 1)
		{
			LED_ON();
			// target_speed += 0.1f;
			// if(target_speed > 2.0f) target_speed = 2.0f;
		}
		else if(Keynum == 2)
		{
			LED_OFF();
			// target_speed -= 0.1f;
			// if(target_speed < 0.0f) target_speed = 0.0f;
		}
		key_flag = 0;
	}

}
