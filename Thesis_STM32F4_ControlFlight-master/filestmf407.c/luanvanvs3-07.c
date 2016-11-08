/**************************************************************************************/
#include "stm32f4xx.h" //10/9/2014
// #include "diskio.h"
// #include "ff.h"
// #include "integer.h"
// #include "string.h"
// #include "powersd.h"
// #include "stdio.h"
// #include "math.h"
#include "project.h"
/**************************************************************************************/
void Delay(void) 
		{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
		{
   
    }
		}
/**************************************************************************************/
//bien cho ham sd card
FRESULT st_Result;   /* Result code */  
FATFS fatfs;  /* File system object */  
FIL fil;   /* File object */  
DIR dir;   /* Directory object */  
FILINFO fno;  /* File information object */  
UINT *bw;
const TCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
/**************************************************************************************/
int count1=0;
int count2=0;
uint8_t flag=0,flag2=0,flag3=0,flag4=0,flag5=0,chieudai=0;
uint16_t lenght=0,lenght1=0,flag1=0;
int y=0;
uint8_t IMU[80];
uint8_t GGA[100];
uint8_t VTG[100];
uint8_t SD[600];
uint8_t SD1[600];
//check ham sampling GPS
//uint8_t GGA1[]={',',',',0x33,0x36,0x39,0x37,'.',0x33,0x35,0x32,0x34,',',',',',',',',',',',',',',0x37,0x39,0x38,'.',0x35,0x38,','};
uint8_t Buf_USART2[300],Buf_UART4[300];
volatile uint8_t dem ;
uint8_t crc;
int h=0;
int i=0;
uint8_t i1=0 ;
uint16_t l=0,l1=0,l2=0;
int main(void)
{
  MyRCC_Configuration();
  MyGPIO_Configuration();
	PID_Init();
	power ();
	UART4_Configuration(115200);
	USART2_Configuration(460800);
	MyTIM_PWM_Configuration();
	DMA_UART4_Configuration(Buf_UART4,300);
	DMA_USART2_Config(Buf_USART2,300);
	NVIC_Configuration();	
	//EXTI_FPGA_Pa8();
	Delay();
	TIM8->CCR4 = 7500;
	//Delay_100ms();
	//TIM8->CCR4 = 3000; // carlib dong co
	f_mount(0, &fatfs);  
  f_mkdir ("0:/HTDKN2");	
	SysTick_Config(SystemCoreClock/500);
	
	while(1)
	{
			if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))
			{
				flag1++;
				if(flag1==1)
				{
					//st_Result = f_close(&fil);
					if (Alt_PID.Current !=0)	Alt_PID.SetPoint = Alt_PID.Current;
					Roll_PID.SetPoint =0;
					Yaw_PID.SetPoint =Yaw_PID.Current;
					Pitch_PID.SetPoint =Pitch_PID.Current;
					//GPIO_SetBits(GPIOD,GPIO_Pin_12);
					Call_Roll_PID(Roll_PID.SetPoint);	
					//Gent_Pwm_Pitch(-5);
					Call_Pitch_PID(Pitch_PID.SetPoint);
					Call_Yaw_PID(Yaw_PID.SetPoint);
				}
				else if(flag1>1)
				{
					Call_Roll_PID(Roll_PID.SetPoint);	
					//Gent_Pwm_Pitch(-5);
					Call_Pitch_PID(Pitch_PID.SetPoint);
					Call_Yaw_PID(Yaw_PID.SetPoint);
				}
				if(flag1==60000) flag1=2;
			}
			else 
			{
				flag1=0;
			}
			if(flag==1)
			{
				flag=0;
				lenght=y;
				if(lenght>100)
				{
					for(i=0;i<lenght;++i)
					{
						SD[i+lenght1]=Buf_USART2[i];
					}
					lenght1+=lenght;
 					st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
// 						if (!st_Result)
// 						{
// 							st_Result = f_lseek(&fil, f_size(&fil));
// 							st_Result = f_write(&fil,SD,lenght1, bw);
// 							st_Result = f_close(&fil); 
// 						}
// 						lenght1=0;
						flag2=1;
				}
				else if((lenght<100)&&(flag2==1))
				{
					flag3++;
					for(i=0;i<80;++i)
					{
						SD[i+lenght1]=Buf_USART2[i];
					}
					lenght1+=80;
					
					if(flag3==4)
					{
						flag3=0;
						flag2=0;
						//st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
						if (!st_Result)
						{
							st_Result = f_lseek(&fil, f_size(&fil));
							st_Result = f_write(&fil,SD,lenght1, bw);
							//st_Result = f_close(&fil); 
						}
						lenght1=0;
						flag4=1;
					}
				}
				else if((lenght<100)&&(flag4==1))
				{
					flag5++;
					for(i=0;i<80;++i)
					{
						SD[i+lenght1]=Buf_USART2[i];
					}
					lenght1+=80;
					 
				}
				if(flag5==4)
				{
					flag5=0;
					flag4=0;
					//st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
					if (!st_Result)
					{
						st_Result = f_lseek(&fil, f_size(&fil));
						st_Result = f_write(&fil,SD,lenght1, bw);
						st_Result = f_close(&fil);
					}
					lenght1=0;
				}
		}
				
// 		if(flag==1)
// 		{
// 			flag=0;
// 			//lenght=y;
// 			lenght=l1+1;
// 			for(i=0;i<lenght;++i)
// 			{
// 				SD[i+lenght1]=Buf_USART2[i];
// 			}
// 			lenght1+=lenght;
// 			if((lenght>100))
// 			{
// 				flag2++;
// 				st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
// 				if (!st_Result)
// 				{
// 					st_Result = f_lseek(&fil, f_size(&fil));
// 					st_Result = f_write(&fil,SD,lenght1, bw);
// 					//st_Result = f_close(&fil); 
// 				}
// 				
// 				if(flag2==1)
// 				{
// 					flag2=0;
// 					st_Result = f_close(&fil);
// 				}
// // 				for(i=0;i<lenght1;++i)
// // 				{
// // 					SD[i]=0;
// // 				}
// 				lenght1=0;
// 				
// 		}
//  	}
	}
}
	
/**************************************************************************************/
void SysTick_Handler(void)
 {	
	
	count1=count2;
	count2=DMA1_Stream5->NDTR;
	if(count1==count2)
	 {
		y=300-count2;
		if (y>10)
		 {
			 //fastest way to restart DMA
			DMA_Cmd(DMA1_Stream5,DISABLE);
			//while( DMA_GetCmdStatus(DMA1_Stream5) );
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
			DMA_SetCurrDataCounter(DMA1_Stream5,300);
			DMA_Cmd(DMA1_Stream5,ENABLE);	
			 if(y>100)
				{			
					for(i=0;i<y;i++)
					{
						dem=0;
						if(Buf_USART2[i]=='$')
						 {
							i1++;
							if(i1==1)
							{
								h=i;//luu vi tri GGA_$															
							}
						 }
						if((Buf_USART2[i]==13)&&(i1==1))
							{
								l=i;//luu vi tri GGA_13
							}
						if((Buf_USART2[i]==13)&&(i1==2))
						{
							l1=i;//luu vi tri VTG_13
							i1=0;
							break;
						}
					 }
					 for(i=0;i<80;i++)
					{
						IMU[i]=Buf_USART2[i];
					}				
					for(i=0;i<l-h+2;i++)
					{
						GGA[i]=Buf_USART2[i+h-1];
					}
					for(i=0;i<l1-l;i++)
					{
						VTG[i]=Buf_USART2[l+1+i];
					}
					Sampling_RPY(Buf_USART2,80);
					Sampling_GGA(GGA ,l-h+2);
					//Sampling_GGA(GGA1 ,26);
					Sampling_VTG(VTG,l1-l);
					flag=1;
					//imu ( DATA[0--> h-2]  BAO GOM \N VA \S\R)
					Buf_UART4[0]=10;
					//Buf_UART4[1]=crc;
					Buf_UART4[2]=32;
					Buf_UART4[3]=73;//I
					Buf_UART4[4]=32;
					for(i=1;i<h-3;i++)
					{
						Buf_UART4[i+4]=Buf_USART2[i];	
					}
					Buf_UART4[h+1]=13;   
					crc=CRC_Cal(3,Buf_UART4,h);	
					Buf_UART4[1]=crc;
	
					//GGA ( DATA[h-1-->l] BAO GOM \N VA \R )
					crc=CRC_Cal(h,Buf_USART2,l-1);
					Buf_UART4[h+2]=10;
					Buf_UART4[h+3]=crc;
					Buf_UART4[h+4]=32;
					for(i=h;i<l+1;i++)
					{
						Buf_UART4[i+5]=Buf_USART2[i];
					}
					
					//VTG ( DATA[l+1 --> x-1] bao gom \n va \r )
					crc=CRC_Cal(l+2,Buf_USART2,y-2);
					Buf_UART4[l+6]=10;
					Buf_UART4[l+7]=crc;
					Buf_UART4[l+8]=32;
					for(i=l+2;i<y;i++)
					{
							Buf_UART4[i+7]=Buf_USART2[i];
					}
				
					DMA_SetCurrDataCounter(DMA1_Stream4,y+7);
					DMA_Cmd(DMA1_Stream4,ENABLE);	
					chieudai=l1+1;
				}
					
				else 
				{	
					dem++;
					Sampling_RPY(Buf_USART2,80);
 					//if(dem%2==0) flag=1;
					flag=1;
					if (dem==6)
					{
						//flag=1;
						//imu ( DATA[0--> y-1]  BAO GOM \N VA \S\R)
						Buf_UART4[0]=10;
						//Buf_UART4[1]=crc;
						Buf_UART4[2]=32;
						Buf_UART4[3]=73;//I
						Buf_UART4[4]=32;
						for(i=1;i<y-2;i++)
						{
							Buf_UART4[i+4]=Buf_USART2[i];	
						}
						Buf_UART4[y+2]=13;   
						crc=CRC_Cal(3,Buf_UART4,y+1);	
						Buf_UART4[1]=crc;
						DMA_SetCurrDataCounter(DMA1_Stream4,y+3);
						DMA_Cmd(DMA1_Stream4,ENABLE);		
					}
				}
			}
			}
}
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
