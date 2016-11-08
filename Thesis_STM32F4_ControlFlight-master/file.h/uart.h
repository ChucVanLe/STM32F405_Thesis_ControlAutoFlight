#ifndef __uart_H
#define __uart_H

#include "stm32f4xx.h"

extern  USART_InitTypeDef USART_InitStructure;
void UART4_Configuration(uint32_t baudrate);
void USART2_Configuration (uint32_t baudrate);
#endif
