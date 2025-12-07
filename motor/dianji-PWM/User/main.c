#include "delay.h"
#include "sys.h"
#include "OLED.h"
#include "led.h"
#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>
#include "DJ.h"
#include "serial.h"
#include "PWM.h"
#include "PID.h"
#include "Key.h"
// #include "test.h"

PID_TypeDef speed_pid;
float target_speed = 0.0f;  //单位m/s

char line_buf[128];
uint16_t line_len = 0;


uint8_t recv_buf[64];
uint16_t lent;

float kp, ki, kd, target;
void Parse_PID_Command(char* line)
{
    float tkp, tki, tkd, ttarget;

    if (sscanf(line,
               "P=%f,I=%f,D=%f,T=%f",
               &tkp, &tki, &tkd, &ttarget) == 4)
    {
        kp = tkp;
        ki = tki;
        kd = tkd;
        target = ttarget;

        // 回显确认
        char msg[64];
        sprintf(msg, "OK P=%.3f I=%.3f D=%.3f T=%.3f\r\n",
                kp, ki, kd, target);
        USART2_DMA_Send((uint8_t*)msg, strlen(msg));
        target_speed = target;
        speed_pid.Kp = kp;
        speed_pid.Ki = ki;
        speed_pid.Kd = kd;
    }
}


void USART_Process(void)
{
    lent = USART2_Getlen();
    if (lent == 0) return;

    USART2_ReadNewData(recv_buf, lent);

    for (uint16_t i = 0; i < lent; i++)
    {
        char c = recv_buf[i];

        /* 拼接到全局缓冲区 */
        if (line_len < sizeof(line_buf) - 1)
            line_buf[line_len++] = c;

        /* 一行结束 */
        if (c == '\n')
        {
            line_buf[line_len] = '\0'; // 加结束符

            Parse_PID_Command(line_buf);

            line_len = 0; // 清空准备下一行
        }
    }
}





extern volatile uint16_t pidout;


int main(void)
{
	sys_stm32_clock_init(9);  // 设置系统时钟为72MHz

	LED_Init();
	Serial2_Init(115200);
    Encoder_Init_TIM3();
	TIM2_OLED_Init();

	PWM_Init();
	TIM5_Update_Init();

	OLED_Init();
	OLED_ColorTurn(0);//0正常显示，1 反色显示	
  	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	OLED_Refresh();
	
	Key_Init();
	TIM6_Update_Init();

	PID_Init(&speed_pid, 0.0f, 0.0f, 0.00f, 300, 900); 

////////////////////////////////////////////
    LED_ON();

////////////////////////////////////////////
	while(1)
	{
		OLED_Clear();
		Show_Speed();
        OLED_ShowNum(0,16,pidout,3,16,1);
		OLED_Refresh();

        LED_OFF();
        USART_Process();

		delay_ms(3);   // 防止刷新太快
	}
}


