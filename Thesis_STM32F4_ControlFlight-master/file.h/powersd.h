#ifndef __powersd_H
#define __powersd_H
#include "stm32f4xx.h"
#include "diskio.h"
#define SPI_SD                   SPI1
#define RCC_GPIO_CS              RCC_AHB1Periph_GPIOA
#define GPIO_CS                  GPIOA
#define GPIO_SPI_SD              GPIOA
#define GPIO_Pin_CS              GPIO_Pin_4
#define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5
#define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6
#define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7
#define RCC_SPI_SD               RCC_APB2Periph_SPI1
#define CS_H()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS) 
 /* - for SPI1 and full-speed APB2: 168MHz/2 */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_8
extern SPI_InitTypeDef  SPI_InitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;
void power (void);

#endif

