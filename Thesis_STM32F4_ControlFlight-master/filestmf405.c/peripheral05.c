/**
  ******************************************************************************
  * @file    Peripheral_Config.c
  * @version initial version 1.0.0
  * @date		 18/10/2014	
  * @brief	 This file contain all macro and function C source to comunicate peripheral in my Project : GCS and Altitude Holding for fixed wing aircraft
  ******************************************************************************
  */
	#include "project.h"
	/**************************************************************************************/
	uint64_t TickCnt;
	/**************************************************************************************/
	void Delay_100ms(void) 
	{
				uint32_t m;
				for(m=0; m<16000000; ++m); 
	};
/**************************************************************************************/
	void MyRCC_Configuration(void)
	{
		/* --------------------------- System Clocks Configuration -----------------*/
		/* UART4 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
		/* GPIOB clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		/* GPIOA clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
		/* DMA1 clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		/*Clock for PWM Timer 4*/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2, ENABLE); 
		/* Enable SD card clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	}
/**************************************************************************************/
	void MyGPIO_Configuration(void)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
	
  
    /* Configure PB12 PB13, pb14, pb15 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
		/*-------------------------- GPIO Configuration ----------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PA.0 UART4_TX, potential clash SCLK CS43L22
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* Connect USART pins to AF */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
		
			//cau hinh gpioA cho usart
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			//gan chan Rx Tx
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2 TX 
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // pA3 RX
			// cau hinh exti
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}

	/**************************************************************************************/
	void UART4_Configuration(uint32_t baudrate)
	{
		USART_InitTypeDef USART_InitStructure;
	
		/* USARTx configuration ------------------------------------------------------*/
		/* USARTx configured as follow:
					- BaudRate = 115200 baud
					- Word Length = 8 Bits
					- One Stop Bit
					- No parity
					- Hardware flow control disabled (RTS and CTS signals)
					- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = baudrate;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
		USART_Init(UART4, &USART_InitStructure);
	
		USART_Cmd(UART4, ENABLE);
	}
	/**************************************************************************************/
	void USART2_Configuration (uint32_t baudrate)
	{
		USART_InitTypeDef USART_InitStructure;
	
		/* USARTx configuration ------------------------------------------------------*/
		/* USARTx configured as follow:
					- BaudRate =460800
					- Word Length = 8 Bits
					- One Stop Bit
					- No parity
					- Hardware flow control disabled (RTS and CTS signals)
					- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = baudrate;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
		USART_Init(USART2, &USART_InitStructure);
	
		USART_Cmd(USART2, ENABLE);
	}
/**************************************************************************************/
	void DMA_USART2_Config(uint8_t *buffer, uint16_t size) 
	{
			DMA_InitTypeDef  DMA_InitStructure;
			DMA_DeInit(DMA1_Stream5);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
			DMA_InitStructure.DMA_BufferSize = size;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   
			//DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;
			//DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
			//DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream5, &DMA_InitStructure);
			USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
			DMA_Cmd(DMA1_Stream5,ENABLE);
			/* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
			//DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
			/* Enable the DMA RX Stream */
	}
	
/**************************************************************************************/
	void DMA_UART4_Configuration(uint8_t *buffer, uint16_t size)
	{
			DMA_InitTypeDef  DMA_InitStructure;
			
			DMA_DeInit(DMA1_Stream4);
			
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
			DMA_InitStructure.DMA_BufferSize = size;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			
			DMA_Init(DMA1_Stream4, &DMA_InitStructure);
			
			/* Enable the USART Tx DMA request */
			USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
			
			/* Enable DMA Stream Transfer Complete interrupt */
			DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
			
			/* Enable the DMA Tx Stream */
			DMA_Cmd(DMA1_Stream4, DISABLE);
	}			
/**************************************************************************************/
void DMA1_Stream4_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		
  }
}

//Ngat nhan dmauart4
void DMA1_Stream5_IRQHandler(void)
{

		/* Clear the DMA1_Stream2 TCIF2 pending bit */
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);//DMA_IT_TCIF2 //co de bao day chua

}
/**************************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  /* Enable the UART4 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************************************************************************/

void DMA_UART4_RX(uint8_t *buffer, uint16_t size)
{
  DMA_InitTypeDef  DMA_InitStructure;
 
  DMA_DeInit(DMA1_Stream2);
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
  DMA_InitStructure.DMA_BufferSize =size;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
 
  /* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
  //DMA_ITConfig(DMA1_Stream2, DMA_IT_HT, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMA1_Stream2, ENABLE);
}
/**************************************************************************************/
//cau hinh ngat dma nhan
void Interrupt_uart4_rx(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  /* Enable the UART4 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************/
	uint64_t GetTickCount(void)
	{
			return TickCnt;
		
	}
/**************************************************************************************/
	uint8_t CheckTick(uint64_t TickBase, uint64_t Time)
	{
			uint64_t CurTick;
			
			CurTick = GetTickCount();
			
			if (CurTick > TickBase)
			{
					if (CurTick >= (TickBase + Time))
					{
							return 1;
					}
			}
			else
			{
					if (CurTick >= ((uint64_t)(TickBase + Time)))
					{
							return 1;
					}
			}
	
			return 0;
	}

/**************************************************************************************/
uint8_t CRC_Cal(uint8_t offsetmin,uint8_t *kitu,int offsetmax)
{
		uint8_t crc,crclsb;
		int j=0;
		int i=offsetmin;
								crc = 0xFF;
						//  byte CRC_LSB;
							for(i=offsetmin; i<=offsetmax;i++)//BO 10 VA 13
							{
									crc ^= *(kitu+i); //XOR
									for (j=0; j<8; j++)
									{
											crclsb = (crc & 0x01);
											crc = ((crc>>1) & 0x7F);
											if (crclsb == 1)
											{
													crc ^= 0xE0;
											}
									}
							}
							
						return crc & 0xff;
		
	}
/**************************************************************************************/
void MyTIM_PWM_Configuration(void)
  { 
		TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
		GPIO_InitTypeDef           GPIO_InitStructure;
		TIM_OCInitTypeDef          TIM_OCInitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
    GPIO_PinAFConfig(GPIOB , GPIO_PinSource6, GPIO_AF_TIM4); 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
    
    /* Time base configuration */
    //TIM_TimeBaseStructure.TIM_Prescaler = 0;  
    //TIM_TimeBaseStructure.TIM_Period = 0xFFFF;// 65535
//		TIM_TimeBaseStructure.TIM_Prescaler =((SystemCoreClock/2)/1000000)-1; //timer 1 nen Fc=2Mhz;
//		TIM_TimeBaseStructure.TIM_Period = 16000;//chu ki 8ms==>moi muc la 0.5microgiay.
 		TIM_TimeBaseStructure.TIM_Prescaler = 53-1;//Fc=5Mhz==>T=2*10^(-4) ms
		TIM_TimeBaseStructure.TIM_Period = 21711-1;//f = 73Hz similar Remote control
// 		TIM_TimeBaseStructure.TIM_Prescaler =((SystemCoreClock/8)/625000)-1;//Fc=2.5Mhz==>T=4*10^(-4) ms
// 		TIM_TimeBaseStructure.TIM_Period = 20000;
//  		TIM_TimeBaseStructure.TIM_Prescaler =((SystemCoreClock/4)/1000000)-1;//Fc=5Mhz==>T=2*10^(-4) ms
// 		TIM_TimeBaseStructure.TIM_Period = 16000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    //TIM_OCStructInit(&TIM_OCInitStructure);
    
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM4->CCR1 = 2377;//pwm 1.5ms
		TIM4->CCR2 = 2377;
		TIM4->CCR3 = 1585;
		TIM4->CCR4 = 2377;
    /* TIM1 enable counter */
    TIM_Cmd(TIM4, ENABLE);
	//timer 1 va 8 moi dung dong nay
    //TIM_CtrlPWMOutputs(TIM4, ENABLE);
  }
	
	/*******************************EXTI**********************/
void EXTI_FPGA_Pa8(void)
  {
		NVIC_InitTypeDef  NVIC_InitStructure;
		EXTI_InitTypeDef  EXTI_InitStructure;
   /* Connect EXTI Line5, Line6 to PB5,PB6 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);
  
    /* Configure EXTI  Line8 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  
    /* Enable and set EXTI Line5, Line6 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
// 	
	

	
void EXTI9_5_IRQHandler(void)
  {
    if(EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
		//get setpoint
		Roll_PID.SetPoint = Roll_PID.Current;
		//reset variable
		Roll_PID.PartKi = 0;
		Roll_PID.PreError = 0;
		Roll_PID.Pid_Result = 0;
		//get setpoint
		Pitch_PID.SetPoint = Pitch_PID.Current;
		//reset variable
		Pitch_PID.PartKi = 0;
		Pitch_PID.PreError = 0;
		Pitch_PID.Pid_Result = 0;
		//get setpoint
		Yaw_PID.SetPoint = Yaw_PID.Current;
		//reset variable
		Yaw_PID.PartKi = 0;
		Yaw_PID.PreError = 0;
		Yaw_PID.Pid_Result = 0;
		//get setpoint
		Alt_PID.SetPoint = Alt_PID.Current;
		//reset variable
		Alt_PID.PartKi = 0;
		Alt_PID.PreError = 0;
		Alt_PID.Pid_Result = 0;
	/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line8);
	  }

  }
