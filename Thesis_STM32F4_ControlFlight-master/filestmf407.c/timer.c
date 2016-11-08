#include "timer.h"
//#include "stm32f4xx_tim.h"
void TIM_PWM_Configuration(void)
  { 
		TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
		GPIO_InitTypeDef           GPIO_InitStructure;
		TIM_OCInitTypeDef          TIM_OCInitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
    GPIO_PinAFConfig(GPIOC , GPIO_PinSource6, GPIO_AF_TIM8); 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
    
    /* Time base configuration */
    //TIM_TimeBaseStructure.TIM_Prescaler = 0;  
    //TIM_TimeBaseStructure.TIM_Period = 0xFFFF;// 65535
		TIM_TimeBaseStructure.TIM_Prescaler =((SystemCoreClock/2)/1000000)-1; //timer 1 nen Fc=2Mhz;
    TIM_TimeBaseStructure.TIM_Period = 16000;//chu ki 8ms;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    //TIM_OCStructInit(&TIM_OCInitStructure);
    
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
		
		TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
    TIM_ARRPreloadConfig(TIM8, ENABLE);
  
    /* TIM1 enable counter */
    TIM_Cmd(TIM8, ENABLE);
	//timer 1 va 8 moi dung dong nay
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
  }
	
	
	
	
	
	
	