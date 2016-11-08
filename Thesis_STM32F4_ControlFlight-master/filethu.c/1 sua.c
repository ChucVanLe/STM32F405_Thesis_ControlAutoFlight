#include "stm32f4xx.h" //10/6/2014
#include "crc.h"
	 void Delay(void) 
{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
		{
   
    }
}

void gpioconfigure (void)
{
		GPIO_InitTypeDef gpioInit;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    //khoi tao GPIOD
    gpioInit.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    gpioInit.GPIO_Mode=GPIO_Mode_OUT;
    gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
    gpioInit.GPIO_OType=GPIO_OType_PP;
    gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &gpioInit);
		//cau hinh gpioA cho usart
		gpioInit.GPIO_Mode=GPIO_Mode_AF;
    gpioInit.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
    gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
    gpioInit.GPIO_OType=GPIO_OType_PP;
    gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpioInit);
		//gan chan Rx Tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2 TX 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // pA3 RX
		//cau hinh uart3
		gpioInit.GPIO_Mode=GPIO_Mode_AF;
    gpioInit.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
    gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
    gpioInit.GPIO_OType=GPIO_OType_PP;
    gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpioInit);
		//GAN CHAN TX RX
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); /// PB10TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);// PB11 RX
	}
	void timer (void)
	{
		// cau hinh timer chia khoang thoi gian
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   //TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;     /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 84*10-1;//1MHz
		TIM_TimeBaseStructure.TIM_Period = 50000-1;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, DISABLE);
  }
	
void USART2_configure (void)
{		
		//NVIC_InitTypeDef nvicInit;
		USART_InitTypeDef usartInit;
	  //cau hinh cho USART2 truyen nhan DMA voi IMU/GPS
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    usartInit.USART_BaudRate=460800;
    usartInit.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    usartInit.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    usartInit.USART_Parity=USART_Parity_No;
    usartInit.USART_StopBits=USART_StopBits_1;
    usartInit.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART2, &usartInit);
		USART_Cmd(USART2, ENABLE);
}
void USART3_configure (void)
{		
		//NVIC_InitTypeDef nvicInit;
		USART_InitTypeDef usartInit;
	  //cau hinh cho USART3 truyen nhan voi may tinh / RF
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    usartInit.USART_BaudRate=115200;
    usartInit.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    usartInit.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    usartInit.USART_Parity=USART_Parity_No;
    usartInit.USART_StopBits=USART_StopBits_1;
    usartInit.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART3, &usartInit);
		USART_Cmd(USART3, ENABLE);
}

uint8_t Buffer[300];	
//int Buffer[400];	
void DMA_Config() 
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
uint8_t buffer[300];	
uint8_t crc;

	void DMA1_config()
{
	NVIC_InitTypeDef NVICInit;
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(DMA1_Stream3);
	// DMA1 clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
 DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(buffer);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  /* Enable the UART2 Tx DMA Interrupt */
  NVICInit.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVICInit.NVIC_IRQChannelPreemptionPriority = 0;
  NVICInit.NVIC_IRQChannelSubPriority = 0;
  NVICInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVICInit);
	
  /* Enable the USART Tx DMA request */
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
 
  /* Enable DMA Stream Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
 
  /* Disable the DMA Tx Stream */
  DMA_Cmd(DMA1_Stream3, DISABLE);
}

int DMA_Done=0;
uint8_t data[500];
uint8_t data2[500];
uint8_t data1[500];
uint8_t data3[500];
uint64_t TickCnt;
uint64_t TickCnt1;
uint64_t count;
int count1=0;
int count2=0;
int count3=0;
uint64_t delay;
uint64_t delay1;
int32_t nowtime =0;
int i=0;
int y=0;
int k=0;
int k1=0;
int k2=0;
int i1=0;
int i2=0;
int i3=0;
int i4=0;
int l=0;
int h=0;
int j=0;
int j1=0;
int x=0;
int flag=0;
int flag1=0;
int flag2=0;
int flag3=0;
int flag4=0;
int flag5=0;
int flag6=0;	
uint64_t GetTickCount(void)
{
    return TickCnt;
	
}

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
 
void transbuff()
{
	int i5;
	DMA_Cmd(DMA1_Stream3, ENABLE);
	while(DMA_Done!=SET);
	DMA_Done = RESET;
	for(i5=0;i5<500;i5++)
	buffer[i5]=0;
}

int main (void)
{
	 

	Delay(); 
	timer ();
	gpioconfigure ();
	USART2_configure ();
	USART3_configure();
	DMA_Config();
	DMA1_config();
  SysTick_Config(SystemCoreClock/1000);

while(1)
 {
	  
 	 if(flag==1) // xu li 3 chuoi 
			{
			
			
		 /// copy du lieu vo data
				for(i=0;i<y;i++)
					{
						data[i]=Buffer[i];
					}
					x=y;
						DMA_Config();
			//xac dinh chuoi
			for(i=0;i<x;i++)
					{
						if(data[i]=='$')
							{
								i1++;
								if(i1==1)
									{
										i2=1;
										h=i;
									}
							}
						if((data[i]==13)&&(i2==1))
							{
								i3++;
								if(i3==1)
									{
										l=i;
									}
							}
				}
				//imu ( DATA[0--> h-2]  BAO GOM \N VA \S\R)
			  if(i2==1)
			  {
				 //crc=tinhcrc(1,data,h-4);// bo \s 
					
					
				//GGA ( DATA[h-1-->l] BAO GOM \N VA \R )
					crc=tinhcrc(h,data,l-1);
					data2[0]=10;
					data2[1]=crc;
					data2[2]=32;
				for(i=h;i<l+1;i++)
					{
						data2[i-h+3]=data[i];
					}
				//VTG ( DATA[l+1 --> x-1] bao gom \n va \r )
					crc=tinhcrc(l+2,data,x-2);
					data3[0]=10;
					data3[1]=crc;
					data3[2]=32;
				for(i=l+2;i<x;i++)
					{
						data3[i-l+1]=data[i];
					}
				for(i=0;i<x;i++)
					{
						data[i]=0;
					}
				 i2=0;
				 i1=0;
				 i3=0;
				 GPIO_SetBits( GPIOD,GPIO_Pin_12);
					
		 /*xu ly xong chuoi
					IMU data1[0-->h+1]
					GGA data2[0-->L-H +3]
					VTG data3[0-->X-L]
					
					*/
					}
					 for(i=0;i<=l-h+3;i++)
				 {
				  USART_SendData(USART3,(data2[i]&0xff));
					while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
					data2[i]=0;
				 }
				 for(i=0;i<=x-l;i++)
					{
					USART_SendData(USART3,(data3[i]&0xff));
					while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
					data3[i]=0;
					}
					flag=0;
				
				}
	
	 }
 }
 
	
	


void SysTick_Handler(void)
 {
			
			count1=count2;
			count2=DMA1_Stream5->NDTR;
			if(count1==count2)
			{
					y=300-count2;
					if(y>100)
						{
					flag=1;
						}
					else 
						{
							if (flag==0)
							{
							for(i=0;i<y;i++)
								{
									data[i]=Buffer[i];
								}
								x=y-1;
								DMA_Config();
						data1[0]=10;
				//	data1[1]=crc;
						data1[2]=32;
						data1[3]=73;//I
						data1[4]=32;
					for(i=1;i<x-1;i++)
					{
						data1[i+4]=data[i];	
					}
					data1[x+3]=13;   
					crc=tinhcrc(3,data1,x+2);	
					data1[1]=crc;

					 for(i=0;i<=x+3;i++)
							{
								USART_SendData(USART3,(data1[i]&0xff));
								while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
								data1[i]=0;
							}
					
								}
							else
							{
								DMA_Config();
							}
						}
					
				}
	}
 
// void DMA1_Stream3_IRQHandler(void)
// {
//   if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
//    {
//     DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
// 		DMA_Done = 1;
//   }
// }