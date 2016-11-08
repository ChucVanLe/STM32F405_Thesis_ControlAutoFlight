#ifndef __dma_H
#define __dma_H

#include "stm32f4xx.h"

extern DMA_InitTypeDef  DMA_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;

void DMA_USART2_Config(uint8_t *buffer, uint16_t size) ;
void DMA_UART4_Configuration(uint8_t *buffer, uint16_t size);
void DMA1_Stream4_IRQHandler(void);
void NVIC_Configuration(void);
#endif
