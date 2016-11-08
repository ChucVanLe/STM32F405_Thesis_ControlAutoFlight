#ifndef __timer_H
#define __timer_H
#include "stm32f4xx.h"
extern  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;;
extern  GPIO_InitTypeDef           GPIO_InitStructure;
extern  TIM_OCInitTypeDef          TIM_OCInitStructure;
void TIM_PWM_Configuration(void);
#endif
