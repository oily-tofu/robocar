#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "led.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
// int main(void)
// {
//     Serial_Init(115200);        // USART2 初始化（PA2/PA3）

//     uint8_t recv_buf[64];
//     uint16_t len;

//     // 先发一个测试消息
//     uint8_t msg[] = "Hello DMA USART2!\r\n";
//     USART2_DMA_Send(msg, sizeof(msg) - 1);

//     while (1)
//     {
//         len = USART2_GetReceivedLength();  // 查询 DMA 新接收数据长度

//         if (len > 0)
//         {
//             USART2_ReadNewData(recv_buf, len);  // 将新收到的 len 字节拿出来

//             // ★★★ DMA 回显数据 ★★★
//             USART2_DMA_Send(recv_buf, len);
//         }
//     }
// }

//////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t USART2_RX_Buffer[64];   // DMA 放数据的位置
uint16_t USART2_Last_Cnt = 0;   // 上一次接收计数
void Serial2_DMA_RX_Init(void)
{
    /* DMA 通道与 USART2 对应：
       USART2_RX → DMA1_Channel6
    */

    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Channel6);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)USART2_RX_Buffer;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;

    dma.DMA_BufferSize         = sizeof(USART2_RX_Buffer);
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;

    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;

    dma.DMA_Mode               = DMA_Mode_Circular;   // 循环接收
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel6, &dma);

    DMA_Cmd(DMA1_Channel6, ENABLE);
}

void USART2_DMA_TX_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel7);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)0;     // 先不填
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;

    DMA_InitStructure.DMA_BufferSize         = 0;               // 先不填
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;

    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}



/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial2_Init(uint32_t baud)
{
    /* --- 1. 使能时钟 --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // PA2/PA3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);     // DMA1

    /* --- 2. 配置 GPIO --- */
    GPIO_InitTypeDef gpio;
    
    // PA2: USART2_TX 复用推挽
    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // PA3: USART2_RX 浮空输入
    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* --- 3. USART2 配置 --- */
    USART_InitTypeDef us;
    us.USART_BaudRate            = baud;
    us.USART_WordLength          = USART_WordLength_8b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &us);
    USART_Cmd(USART2, ENABLE);

    /* 打开 DMA 收发 */
    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    Serial2_DMA_RX_Init();
    USART2_DMA_TX_Init();
    
}




uint8_t USART2_DMA_Send(uint8_t *buf, uint16_t len)
{
    if(DMA_GetCurrDataCounter(DMA1_Channel7) != 0)
        return 0; // DMA 忙，直接返回

    DMA_DeInit(DMA1_Channel7);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize         = len;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel7, ENABLE);

    return 1;
}



uint16_t USART2_Getlen(void)
{
    uint16_t now_cnt = DMA_GetCurrDataCounter(DMA1_Channel6);
    uint16_t len;

    if (now_cnt <= USART2_Last_Cnt)
    {
        // 正常情况，收了新数据
        len = USART2_Last_Cnt - now_cnt;
    }
    else
    {
        // 缓冲区环绕一圈
        len = USART2_Last_Cnt + (64 - now_cnt);
    }

    USART2_Last_Cnt = now_cnt;
    return len;
}

void USART2_ReadNewData(uint8_t* out, uint16_t len)
{
    static uint16_t read_pos = 0;  // DMA 写的位置不断绕圈

    for (uint16_t i = 0; i < len; i++)
    {
        out[i] = USART2_RX_Buffer[read_pos];
        read_pos = (read_pos + 1) % 64;   // 读指针绕圈
    }
}

