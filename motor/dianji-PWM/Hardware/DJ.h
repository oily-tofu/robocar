#ifndef __DJ_H
#define __DJ_H

void DJ_Init(void);
uint8_t Read_PA5(void);
uint8_t Read_PA6(void);

void Encoder_Init_TIM3(void);
int16_t Encoder_GetTIM3(void);
int32_t Encoder_GetSpeed(void);

void TIM2_OLED_Init(void);
void Show_Speed(void);

#endif
