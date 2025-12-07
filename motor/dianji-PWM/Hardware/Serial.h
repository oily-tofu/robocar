#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

void Serial2_Init(uint32_t baud);
uint8_t USART2_DMA_Send(uint8_t *buf, uint16_t len);
void Serial2_DMA_RX_Init(void);
uint16_t USART2_Getlen(void);
void USART2_ReadNewData(uint8_t* out, uint16_t len);




#endif
