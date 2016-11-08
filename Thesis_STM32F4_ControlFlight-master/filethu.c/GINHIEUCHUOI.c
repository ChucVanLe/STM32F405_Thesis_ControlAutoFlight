// #include "stm32f4xx.h" //10/6/2014
// #include "crc.h"
// #include "diskio.h"
// #include "ff.h"
// #include "integer.h"
// #include "string.h"
// #include "powersd.h"
// #include "rccmain.h"
// #include "uart.h"
// #include "dma.h"
// /**************************************************************************************/
// void Delay(void) 
// 		{
//     uint32_t m;
//     for(m=0; m<0xffffff; ++m) 
// 		{
//    
//     }
// 		}

// /**************************************************************************************/
// FRESULT st_Result;   /* Result code */  
// FATFS fatfs;  /* File system object */  
// FIL fil;   /* File object */  
// DIR dir;   /* Directory object */  
// FILINFO fno;  /* File information object */  
// UINT *bw;
// const TCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
// int  flag_writedata_sd=0;
// uint16_t Send_Sd_lenght=0;
// int flag_imu=0;
// int a,counter_interupt=0,flag_into_main=0;
// uint8_t 	data_sd[300];
// uint8_t Buffer[300];	
// uint8_t Buffer1[300];		
// uint8_t  buffer[300];
// uint8_t buffer1[300];	
// uint8_t x_IMU[5],y_IMU[5],z_IMU[5];
// uint64_t TickCnt;
// uint64_t TickCnt1;
// uint64_t count;
// int count1=0;
// int count2=0;
// uint8_t crc;
// int i=0,i_ngat=0;
// int count_$ =0,GGA_$ =0,GGA_13=0;
// int lenght_data_receive=0,lenght_data_receive1=0,lenght_data_receive2=0;
// /**************************************************************************************/
// uint64_t GetTickCount(void)
// {
//     return TickCnt;
// 	
// }
// uint8_t CheckTick(uint64_t TickBase, uint64_t Time)
// {
//     uint64_t CurTick;
//     
//     CurTick = GetTickCount();
//     
//     if (CurTick > TickBase)
//     {
//         if (CurTick >= (TickBase + Time))
//         {
//             return 1;
//         }
//     }
//     else
//     {
//         if (CurTick >= ((uint64_t)(TickBase + Time)))
//         {
//             return 1;
//         }
//     }

//     return 0;
// }

// void GPIO_Configuration(void);
// /**************************************************************************************/
// int main(void)
// {
// 	RCC_Configuration();
// 	power();
// 	NVIC_Configuration();
// 	GPIO_Configuration();
// 	UART4_Configuration(115200);
// 	USART2_Configuration(460800);
// 	DMA_UART4_Configuration(buffer1,300);
// 	DMA_USART2_Config(Buffer,300);
// 	f_mount(0, &fatfs);  
//   f_mkdir ("0:/HTDKN2");	
// 	 
// 	SysTick_Config(SystemCoreClock/1000);
// 	while(1)
// 	{
// 		if(flag_into_main==1)
// 		{
// 			flag_into_main=0;
// 			if(lenght_data_receive1 > 100) //la GPS
// 			{
// 				
// 				for(i=0;i<lenght_data_receive1;i++)
// 				{
// 					if(Buffer1[i]=='$')
// 					{
// 						count_$ ++;
// 						if(count_$==1)
// 						{
// 							GGA_$=i;															
// 						}
// 					}
// 					if((Buffer1[i]==13)&&(count_$==1))
// 					{
// 						GGA_13=i;
// 					}
// 				}
// 				//tinh buffer truyen IMU
// 				buffer1[0]=10;
// 				//buffer[1]=crc;
// 				buffer1[2]=32;
// 				buffer1[3]=73;//I
// 				buffer1[4]=32;
// 				for(i=1;i<GGA_$ - 3;i++)
// 				{
// 					buffer1[i+4]=Buffer1[i];	
// 				}
// 				buffer1[GGA_$+1]=13;   
// 				crc=tinhcrc(3,buffer1,GGA_$);	
// 				buffer1[1]=crc;
// 					
// 				//GGA ( DATA[h-1-->l] BAO GOM \N VA \R )
// 				crc=tinhcrc(GGA_$,Buffer1,GGA_13-1);
// 				buffer1[GGA_$+2]=10;
// 				buffer1[GGA_$+3]=crc;
// 				buffer1[GGA_$+4]=32;
// 				for(i=GGA_$;i<GGA_13+1;i++)
// 					{
// 						buffer1[i+5]=Buffer1[i];
// 					}
// 				//VTG ( DATA[l+1 --> x-1] bao gom \n va \r )
// 				crc=tinhcrc(GGA_13+2,Buffer1,lenght_data_receive1-2);
// 				buffer1[GGA_13+6]=10;
// 				buffer1[GGA_13+7]=crc;
// 				buffer1[GGA_13+8]=32;
// 				for(i=GGA_13+2;i<lenght_data_receive1;i++)
// 				{
// 					buffer1[i+7]=Buffer1[i];
// 				}
// 				count_$=0;
// 				for(i=0;i<lenght_data_receive1+7;i++)
// 				{
// 					buffer[i]=buffer1[i];
// 				}
// 				DMA_SetCurrDataCounter(DMA1_Stream4,lenght_data_receive1+7);
// 				DMA_Cmd(DMA1_Stream4,ENABLE);	
// 				flag_writedata_sd=1;
// 			}
// 		else if(lenght_data_receive2 < 100)
// 			{
// 				//imu ( DATA[0--> y-1]  BAO GOM \N VA \S\R)
// 				buffer1[0]=10;
// 				//buffer[1]=crc;
// 				buffer1[2]=32;
// 				buffer1[3]=73;//I
// 				buffer1[4]=32;
// 				for(i=1;i<lenght_data_receive2-2;i++)
// 				{
// 					buffer1[i+4]=Buffer1[i];	
// 				}
// 				buffer1[lenght_data_receive1+2]=13;   
// 				crc=tinhcrc(3,buffer1,lenght_data_receive2+1);	
// 				buffer1[1]=crc;
// 				for(i=0;i<lenght_data_receive2+3;i++)
// 				{
// 					buffer[i]=buffer1[i];
// 				}
// // 				DMA_SetCurrDataCounter(DMA1_Stream4,lenght_data_receive2+3);
// // 				DMA_Cmd(DMA1_Stream4,ENABLE);	
// 				flag_writedata_sd=1;
// 			}
// 	}
// }
// }
// 	
// /**************************************************************************************/
// void SysTick_Handler(void)
//  {
// 		count1=count2;
// 		count2=DMA1_Stream5->NDTR;
// 		if(count1==count2)
// 		{
// 			lenght_data_receive=300-count2;
// 			if (lenght_data_receive>10)
// 			{
// 				//	fastest way to restart DMA
// 				DMA_Cmd(DMA1_Stream5,DISABLE);
// 				//	while( DMA_GetCmdStatus(DMA1_Stream5) );
// 				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
// 				DMA_SetCurrDataCounter(DMA1_Stream5,300);
// 				DMA_Cmd(DMA1_Stream5,ENABLE);
// 				if(lenght_data_receive>100)
// 				{
// 					counter_interupt=0;
// 					for(i_ngat=0;i_ngat<lenght_data_receive;i_ngat++)
// 					{
// 						Buffer1[i_ngat]=Buffer[i_ngat];
// 						lenght_data_receive1=lenght_data_receive;
// 					}
// 						flag_into_main=1;
// 					}
// 				 else
// 					{
// 						counter_interupt++;
// 						if(counter_interupt%3==0)
// 						{
// 							for(i_ngat=0;i_ngat<lenght_data_receive;i_ngat++)
// 							{
// 							Buffer1[i_ngat]=Buffer[i_ngat];
// 							lenght_data_receive2=lenght_data_receive;
// 							}
// 							flag_into_main=1;
// 						}
// 					}
// 				}
// 			}
// 		}			
// 	

