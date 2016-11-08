#include "stm32f4xx.h" //10/6/2014
#include "crc.h"
#include "diskio.h"
#include "ff.h"
#include "integer.h"
#include "string.h"
/**************************************************************************************/
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



FRESULT st_Result;   /* Result code */  
FATFS fatfs;  /* File system object */  
FIL fil;   /* File object */  
DIR dir;   /* Directory object */  
FILINFO fno;  /* File information object */  
UINT *bw;
uint8_t Message1[] ={0x35,0x37};
uint8_t Message2[] ={0x36,0x37};
UCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 



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
	int i;
	RCC_Configuration();
	power();
		
  //RCC_Configuration();
 
  NVIC_Configuration();
 
  GPIO_Configuration();
	
	UART4_Configuration();
	UART2_Configuration();
	
	DMA_UART4_Configuration();
	DMA_USART2_Config();
	  
    
 	
   f_mount(0, &fatfs);  
   f_mkdir ("0:/HTDKN2");	
 	
   st_Result = f_open(&fil, FilePath, FA_WRITE | FA_CREATE_ALWAYS);
   for(i=0;i<400;++i)
   st_Result = f_write(&fil, Message1, 1, bw);
	 st_Result = f_close(&fil); 
	 
	 st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
	 st_Result = f_lseek(&fil, f_size(&fil));
   for(i=0;i<400;++i)
   st_Result = f_write(&fil, Message2, 1, bw);
	 st_Result = f_close(&fil);
	 
	 st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
	 st_Result = f_lseek(&fil, f_size(&fil));
   for(i=0;i<400;++i)
   st_Result = f_write(&fil, Message1, 1, bw);
	 st_Result = f_close(&fil); 

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
							if (dem%6 ==0)
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
					
							DMA_SetCurrDataCounter(DMA1_Stream4,y+3);
							DMA_Cmd(DMA1_Stream4,ENABLE);	
						
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
					if ( dem>9) dem=0;
				}					
			}
	}
}
 void power (void)
{
	volatile static uint32_t  i;
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	volatile uint8_t dummyread;

	/* Enable GPIO clock for CS */
//	RCC_AHB1PeriphClockCmd(RCC_GPIO_CS, ENABLE);
	/* Enable SPI clock, SPI1: APB2, SPI2: APB1 */
	RCC_APB2PeriphClockCmd(RCC_SPI_SD, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
     /* Configure PB0 PB1 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

	//for( i= 1000000;i>0;i--) { ;} /* wait for ready*/

	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_CS, &GPIO_InitStructure);

	/* De-select the Card: Chip Select high */
	CS_H();

	/* Configure SPI pins: SCK and MOSI with default alternate function (not re-mapped) push-pull */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_SCK | GPIO_Pin_SPI_SD_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);
	/* Configure MISO as Input with internal pull-up */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 72000kHz/256=281kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI_SD, &SPI_InitStructure);
	SPI_CalculateCRC(SPI_SD, DISABLE);
	SPI_Cmd(SPI_SD, ENABLE);

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);   // only connect to 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);   // only connect to 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);   // only connect to 
        
	/* drain SPI */
	//while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_TXE) == RESET) { ; }
	//dummyread = SPI_I2S_ReceiveData(SPI_SD);

}
/**************************************************************************************/
/**************************************************************************************/
