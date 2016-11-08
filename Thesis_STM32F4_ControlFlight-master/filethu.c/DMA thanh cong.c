#include "stm32f4xx.h" //10/6/2014
#include "crc.h"
/**************************************************************************************/
uint8_t Buffer[300];	
uint8_t  buffer[300];
uint64_t TickCnt;
uint64_t TickCnt1;
uint64_t count;
int count1=0;
int count2=0;
int y=0;

uint8_t dem ;
/**************************************************************************************/
void Delay(void) 
		{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
		{
   
    }
		}
/**************************************************************************************/
		void RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* UART4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 
  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 
  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
}
/**************************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC.10 UART4_TX, potential clash SCLK CS43L22
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
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
}

/**************************************************************************************/
void UART4_Configuration(void)
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
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(UART4, &USART_InitStructure);
 
  USART_Cmd(UART4, ENABLE);
}
/**************************************************************************************/
void UART2_Configuration(void)
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
  USART_InitStructure.USART_BaudRate = 460800;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(USART2, &USART_InitStructure);
 
  USART_Cmd(USART2, ENABLE);
}
/**************************************************************************************/
void DMA_USART2_Config() 
{
		DMA_InitTypeDef  DMA_InitStructure;
		DMA_DeInit(DMA1_Stream5);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
		DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(Buffer);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;   
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
	void DMA_UART4_Configuration(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
 
  DMA_DeInit(DMA1_Stream4);
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(buffer) - 1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
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
int main(void)
{
  RCC_Configuration();
 
  NVIC_Configuration();
 
  GPIO_Configuration();
	
	UART4_Configuration();
	UART2_Configuration();
	
	DMA_UART4_Configuration();
	DMA_USART2_Config();
	
	SysTick_Config(SystemCoreClock/1000);
	while(1);
}
	
/**************************************************************************************/
void SysTick_Handler(void)
 {
		
			uint8_t crc;
			int h;
			int i=0;
			uint8_t i1=0 ;
			int l=0;
			count1=count2;
			count2=DMA1_Stream5->NDTR;
			if(count1==count2)
			{
			
				y=300-count2;
				if (y>10)
				{
					if(y>100)
						{								
									//	fastest way to restart DMA
									DMA_Cmd(DMA1_Stream5,DISABLE);
									//	while( DMA_GetCmdStatus(DMA1_Stream5) );
									DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
									DMA_SetCurrDataCounter(DMA1_Stream5,300);
									DMA_Cmd(DMA1_Stream5,ENABLE);	
	
									for(i=0;i<y;i++)
									{
											if(Buffer[i]=='$')
											{
													i1++;
													if(i1==1)
														{
															h=i;															
														}
											}
											if((Buffer[i]==13)&&(i1==1))
											{
															l=i;
											}
									}
									
									
									//imu ( DATA[0--> h-2]  BAO GOM \N VA \S\R)
									buffer[0]=10;
									//	buffer[1]=crc;
									buffer[2]=32;
									buffer[3]=73;//I
									buffer[4]=32;
									for(i=1;i<h-3;i++)
									{
											buffer[i+4]=Buffer[i];	
									}
									buffer[h+1]=13;   
									crc=tinhcrc(3,buffer,h);	
									buffer[1]=crc;
					
									//GGA ( DATA[h-1-->l] BAO GOM \N VA \R )
									crc=tinhcrc(h,Buffer,l-1);
									buffer[h+2]=10;
									buffer[h+3]=crc;
									buffer[h+4]=32;
									for(i=h;i<l+1;i++)
									{
											buffer[i+5]=Buffer[i];
									}
									
									//VTG ( DATA[l+1 --> x-1] bao gom \n va \r )
									crc=tinhcrc(l+2,Buffer,y-2);
									buffer[l+6]=10;
									buffer[l+7]=crc;
									buffer[l+8]=32;
									for(i=l+2;i<y;i++)
									{
											buffer[i+7]=Buffer[i];
									}
			
				
									i1=0;
	
					
									/*xu ly xong chuoi
									buffer [0..y+6]   lengt y+7
									*/
									
									//Send DMA
//									for(i=h+4;i<=y+8;i++)
//									{
//										USART_SendData(UART4,(buffer[i]&0xff));
//										while(USART_GetFlagStatus(UART4,USART_FLAG_TXE) == RESET){}
//									}
									DMA_SetCurrDataCounter(DMA1_Stream4,y+7);
									DMA_Cmd(DMA1_Stream4,ENABLE);	
						}
					
						
						else 
						{	
							dem++;
							if (dem%4 ==0)
							{
							//	fastest way to restart DMA
							DMA_Cmd(DMA1_Stream5,DISABLE);
							//	while( DMA_GetCmdStatus(DMA1_Stream5) );
							DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
							DMA_SetCurrDataCounter(DMA1_Stream5,300);
							DMA_Cmd(DMA1_Stream5,ENABLE);

							//imu ( DATA[0--> y-1]  BAO GOM \N VA \S\R)
							buffer[0]=10;
						//	buffer[1]=crc;
							buffer[2]=32;
							buffer[3]=73;//I
							buffer[4]=32;
							for(i=1;i<y-2;i++)
							{
									buffer[i+4]=Buffer[i];	
							}
							buffer[y+2]=13;   
							crc=tinhcrc(3,buffer,y+1);	
							buffer[1]=crc;
							
							
							//send data
//							 for(i=0;i<=y+2;i++)
//						{
//										USART_SendData(UART4,(buffer[i]&0xff));
//										while(USART_GetFlagStatus(UART4,USART_FLAG_TXE) == RESET){}
//							}
					
						//	DMA_SetCurrDataCounter(DMA1_Stream4,y+3);
						//	DMA_Cmd(DMA1_Stream4,ENABLE);	
						
						}	
					
					else
					{
							//	fastest way to restart DMA
							DMA_Cmd(DMA1_Stream5,DISABLE);
							//	while( DMA_GetCmdStatus(DMA1_Stream5) );
							DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
							DMA_SetCurrDataCounter(DMA1_Stream5,300);
							DMA_Cmd(DMA1_Stream5,ENABLE);
					}
					if ( dem>100) dem=0;
				}					
			}
	}
}
/**************************************************************************************/
/**************************************************************************************/
