#include "stm32f4xx.h" //10/6/2014
#include "crc.h"
#include "diskio.h"
#include "ff.h"
#include "integer.h"
#include "string.h"
#include "powersd.h"
#include "rccmain.h"
#include "uart.h"
#include "dma.h"
/**************************************************************************************/
void Delay(void) 
		{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
		{
   
    }
		}

/**************************************************************************************/
FRESULT st_Result;   /* Result code */  
FATFS fatfs;  /* File system object */  
FIL fil;   /* File object */  
DIR dir;   /* Directory object */  
FILINFO fno;  /* File information object */  
UINT *bw;
const TCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
int  flag_writedata_sd=0;
uint16_t Send_Sd_lenght=0;
int flag_imu=0;
int a,couter_interupt;
uint8_t 	data_sd[300];
uint8_t Buffer[300];	
uint8_t  buffer[300];
uint64_t TickCnt;
uint64_t TickCnt1;
uint64_t count;
int count1=0;
int count2=0;
int lenght_data_receive=0,h;
/**************************************************************************************/
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

void GPIO_Configuration(void);
/**************************************************************************************/
int main(void)
{
	RCC_Configuration();
	power();
	NVIC_Configuration();
	GPIO_Configuration();
	UART4_Configuration(115200);
	USART2_Configuration(460800);
	DMA_UART4_Configuration(buffer,300);
	DMA_USART2_Config(Buffer,300);
	f_mount(0, &fatfs);  
  f_mkdir ("0:/HTDKN2");	
	 
	SysTick_Config(SystemCoreClock/1000);
	while(1)
	{
		if(flag_writedata_sd == 1)
		{
			flag_writedata_sd=0;
			st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
			if (!st_Result)
							{
							 st_Result = f_lseek(&fil, f_size(&fil));
							 st_Result = f_write(&fil, data_sd,Send_Sd_lenght, bw);
							 st_Result = f_close(&fil); 
								flag_imu =1;
							}
							
		}
	}
}
	
/**************************************************************************************/
void SysTick_Handler(void)
 {
		
			uint8_t crc;
			int i=0;
			int i1=0 ;
			int l=0;
			count1=count2;
			count2=DMA1_Stream5->NDTR;
			if(count1==count2)
			{
				lenght_data_receive=300-count2;
				if (lenght_data_receive>10)
				{
					if(lenght_data_receive>100) //la chuoi GPS
						{
							couter_interupt=0;
							//	fastest way to restart DMA
							DMA_Cmd(DMA1_Stream5,DISABLE);
							//	while( DMA_GetCmdStatus(DMA1_Stream5) );
							DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
							DMA_SetCurrDataCounter(DMA1_Stream5,300);
							DMA_Cmd(DMA1_Stream5,ENABLE);	
									
							for(i=0;i<lenght_data_receive;i++)
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
							//buffer[1]=crc;
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
							crc=tinhcrc(l+2,Buffer,lenght_data_receive-2);
							buffer[l+6]=10;
							buffer[l+7]=crc;
							buffer[l+8]=32;
							for(i=l+2;i<lenght_data_receive;i++)
							{
								buffer[i+7]=Buffer[i];
							}
							i1=0;
	
							DMA_SetCurrDataCounter(DMA1_Stream4,lenght_data_receive+7);
							DMA_Cmd(DMA1_Stream4,ENABLE);	
							if (flag_writedata_sd==0)
							{
							for(i=0;i<lenght_data_receive;i++)
								{
									data_sd[i]=Buffer[i];
								}
							Send_Sd_lenght =lenght_data_receive;
							flag_writedata_sd=1;
							}
					}
					else
					{

							couter_interupt++;
							if(couter_interupt%2==0)
							{
								buffer[2]=32;
								buffer[3]=73;//I
								buffer[4]=32;
								for(i=1;i<lenght_data_receive-2;i++)
								{
									buffer[i+4]=Buffer[i];	
								}
								buffer[lenght_data_receive+2]=13;   
								crc=tinhcrc(3,buffer,lenght_data_receive+1);	
								buffer[1]=crc;
								
							if (flag_writedata_sd==0)
							{
							for(i=0;i<lenght_data_receive;i++)
							{
								data_sd[i]=Buffer[i];
							}
								Send_Sd_lenght =lenght_data_receive;
								flag_writedata_sd=1;
							}
						
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
						}

			}
	}
}


/**************************************************************************************/
/**************************************************************************************/
