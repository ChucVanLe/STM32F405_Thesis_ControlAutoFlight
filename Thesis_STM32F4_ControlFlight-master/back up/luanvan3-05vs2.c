/**************************************************************************************/
//3/12/2014:lay mau 20ms PID,ghi 5 chuoi IMU,2 chuoi GPS,PID cac goc canh,dong co cho quay 50%
// #include "diskio.h"
// #include "ff.h"
// #include "integer.h"
// #include "string.h"
// #include "powersd.h"
// #include "stdio.h"
// #include "math.h"
//check ham sampling GPS
//uint8_t GGA1[]={',',',',0x33,0x36,0x39,0x37,'.',0x33,0x35,0x32,0x34,',',',',',',',',',',',',',',0x37,0x39,0x38,'.',0x35,0x38,','};

#include "project.h"
/**************************************************************************************/

/**************************************************************************************/
//bien cho ham sd card
FRESULT st_Result = FR_OK;   /* Result code */  
FATFS fatfs;  /* File system object */  
FIL fil;   /* File object */  
DIR dir;   /* Directory object */  
FILINFO fno;  /* File information object */  
UINT *bw;
const TCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
/**************************************************************************************/
uint8_t CMD_Trigger =0;
uint8_t Alt_latest =0;
int them=0,them1=0;//bien dem cho thoi gian ngat truyen.
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
uint8_t CMD_delay=0;
uint8_t G_delay_count=0;
uint8_t Buf_USART2[300],Buf_UART4[300],Buf_rx4[9],Buf1_rx4[9];
uint8_t Update_heso_Roll=0,Update_heso_Pitch=0,Update_heso_Yaw=0,Update_heso_Alt=0,Update_heso_Press=0;
volatile uint8_t dem=0;
uint8_t crc;
int h=0;
int i=0;
uint8_t i1=0 ;
uint16_t l=0,l1=0,l2=0;
uint8_t flag_set_or_current_press;
uint8_t flag_press=0;
uint8_t state_alt=1,state_press=0;
void xacnhan(uint8_t *buffer);
/*************************************************************************************/
int main(void)
{
	Delay_100ms();
  MyRCC_Configuration();
	Delay_100ms();
	NVIC_Configuration();	
	Interrupt_uart4_rx();
	PID_Init();
  MyGPIO_Configuration();
	EXTI_FPGA_Pa8();
	UART4_Configuration(115200);
	USART2_Configuration(460800);
	DMA_UART4_Configuration(Buf_UART4,300);
	DMA_USART2_Config(Buf_USART2,300);
	DMA_UART4_RX(Buf_rx4,9);
// defaude dieu khien o che do ALT	
	state_press=0;
	state_alt=1;


	power ();
	
	MyTIM_PWM_Configuration();	
	
	f_mount(0, &fatfs);  
  f_mkdir ("0:/HTDKN2");	
	SysTick_Config(SystemCoreClock/500);
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);
// 	GPIO_SetBits(GPIOB,GPIO_Pin_13);
// 	GPIO_SetBits(GPIOB,GPIO_Pin_14);
// 	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	while(1)
	{
		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))
		{
				Call_Roll_PID(Roll_PID.SetPoint);	
				//Gent_Pwm_Pitch(-5);
				Call_Pitch_PID(Pitch_PID.SetPoint);
				Call_Yaw_PID(Yaw_PID.SetPoint);
				Call_Alt_PID(Alt_PID.SetPoint);
		}
		
// /*.................................ghi 5 chuoi...........................................*/				
		if(flag==8)
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
						
					flag2=1;
				}
				else
				{
					if(flag2==1)
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
							st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
							if (!st_Result)
							{
								st_Result = f_lseek(&fil, f_size(&fil));
								st_Result = f_write(&fil,SD,lenght1, bw);
								st_Result = f_close(&fil); 
							}
						lenght1=0;
						}
					 }
				 }
			}				
	}// end while
}//end main
	
/**************************************************************************************/
void SysTick_Handler(void)
 {	
	count1=count2;
	count2=DMA1_Stream5->NDTR;
	if( CMD_delay ==1 )
	{	
		G_delay_count +=1;
	}
	if (G_delay_count ==200)
	{
		G_delay_count =0;
		CMD_delay =0;
		DMA_UART4_RX(Buf_rx4,9);
	}
		
		
		
	if(count1==count2)
	{
		y=300-count2;
		if (y>10)
		{
			 //fastest way to restart DMA
			DMA_Cmd(DMA1_Stream5,DISABLE);
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
//					for(i=0;i<l-h+2;i++)
//					{
//						GGA[i]=Buf_USART2[i+h-1];
//					}
//					for(i=0;i<l1-l;i++)
//					{
//						VTG[i]=Buf_USART2[l+1+i];
//					}
					Sampling_RPY(Buf_USART2,80);
					Sampling_GGA(&Buf_USART2[h-1] ,l-h+2);
					//Sampling_GGA(GGA1 ,26);
					Sampling_VTG(&Buf_USART2[l+1],l1-l);
					flag=1;
					//imu ( DATA[0--> h-2]  BAO GOM \N VA \S\R)
					if ( CMD_Trigger == 0)
					{
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
//							chieudai=l1+1;
						}
						else
						{
							CMD_Trigger=0;
//							DMA_Cmd(DMA1_Stream2,DISABLE);
//							DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
//							DMA_SetCurrDataCounter(DMA1_Stream2,9);
//							DMA_Cmd(DMA1_Stream2,ENABLE);
						}
					
				}
					
				else 
				{	
					dem++;
 					if(dem%2==0) 
					{	
						Sampling_RPY(Buf_USART2,80);
					}
					flag=1;
					
					if (dem==5)
					{
						if ( CMD_Trigger ==0)
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
						else
						{
							CMD_Trigger =0;
//							DMA_Cmd(DMA1_Stream2,DISABLE);
//							DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
//							DMA_SetCurrDataCounter(DMA1_Stream2,9);
//							DMA_Cmd(DMA1_Stream2,ENABLE);
						}
					}
					 
				}
			 }
		}
  }

//Ngat nhan dmauart4
	void DMA1_Stream2_IRQHandler(void)
	{
		/* Test on DMA Stream Transfer Complete interrupt */
		if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
		{
			DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
			CMD_Trigger =1 ;	
			CMD_delay =1 ;
			//tat DMA Tx
			DMA_Cmd(DMA1_Stream4,DISABLE);
			DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
			
			xacnhan(Buf_rx4);
//			them1=1;
						//restart DMA_RX
//			DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
				
		}
	}
void xacnhan(uint8_t *buffer)
	{
		switch (buffer[0])
		{
			case 1:
				
				if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
				{
					Buf_UART4[0]=13;
					Buf_UART4[1]=10;
					Buf_UART4[2]=buffer[0];
					Buf_UART4[3]=6;//ACK
					Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
					Buf_UART4[5]=13;
					DMA_SetCurrDataCounter(DMA1_Stream4,6);
					DMA_Cmd(DMA1_Stream4,ENABLE);
					Roll_PID.Kp=buffer[1];
					Roll_PID.Ki=buffer[2];
					Roll_PID.Kd=buffer[3];
					if(buffer[4]!=0)
					{
					Roll_PID.SetPoint=buffer[4];
					}
					Update_heso_Roll=1;
				}
				else
				{
					Buf_UART4[0]=13;
					Buf_UART4[1]=10;
					Buf_UART4[2]=buffer[0];
					Buf_UART4[3]=21;
					Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
					Buf_UART4[5]=13;
					DMA_SetCurrDataCounter(DMA1_Stream4,6);
					DMA_Cmd(DMA1_Stream4,ENABLE);
				}
				break;
				
			case 2:
					if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
					{
					//	them=1;
						Buf_UART4[0]=13;
						Buf_UART4[1]=10;
						Buf_UART4[2]=buffer[0];
						Buf_UART4[3]=6;//ACK
						Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
						Buf_UART4[5]=13;
						DMA_SetCurrDataCounter(DMA1_Stream4,6);
						DMA_Cmd(DMA1_Stream4,ENABLE);
						Pitch_PID.Kp=buffer[1];
						Pitch_PID.Ki=buffer[2];
						Pitch_PID.Kd=buffer[3];
						if(buffer[4]!=0)
						{
						Pitch_PID.SetPoint=buffer[4];
						}
						Update_heso_Pitch=1;
					}
					else
					{
						Buf_UART4[0]=13;
						Buf_UART4[1]=10;
						Buf_UART4[2]=buffer[0];
						Buf_UART4[3]=21;
						Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
						Buf_UART4[5]=13;
						DMA_SetCurrDataCounter(DMA1_Stream4,6);
						DMA_Cmd(DMA1_Stream4,ENABLE);
					}
					break;
					
					case 3:
						if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
						{
							Buf_UART4[0]=13;
							Buf_UART4[1]=10;
							Buf_UART4[2]=buffer[0];
							Buf_UART4[3]=6;//ACK
							Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
							Buf_UART4[5]=13;
							DMA_SetCurrDataCounter(DMA1_Stream4,6);
							DMA_Cmd(DMA1_Stream4,ENABLE);
							Yaw_PID.Kp=buffer[1];
							Yaw_PID.Ki=buffer[2];
							Yaw_PID.Kd=buffer[3];
							if(buffer[4]!=0)
							{
							Yaw_PID.SetPoint=buffer[4];
							}
							Update_heso_Yaw=1;
						}
						else
						{
							Buf_UART4[0]=13;
							Buf_UART4[1]=10;
							Buf_UART4[2]=buffer[0];
							Buf_UART4[3]=21;
							Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
							Buf_UART4[5]=13;
							DMA_SetCurrDataCounter(DMA1_Stream4,6);
							DMA_Cmd(DMA1_Stream4,ENABLE);
						}
						break;
						
					case 4:
							if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
							{
							Buf_UART4[0]=13;
							Buf_UART4[1]=10;
							Buf_UART4[2]=buffer[0];
							Buf_UART4[3]=6;//ACK
							Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
							Buf_UART4[5]=13;
							DMA_SetCurrDataCounter(DMA1_Stream4,6);
							DMA_Cmd(DMA1_Stream4,ENABLE);
							Alt_PID.Kp=buffer[1];
							Alt_PID.Ki=buffer[2];
							Alt_PID.Kd=buffer[3];
								//if(buffer[4]!=0)  Alt_PID.SetPoint=buffer[4];
							Update_heso_Alt=1;
							state_press=0;
							state_alt=1;
							}
						else
						{
							Buf_UART4[0]=13;
							Buf_UART4[1]=10;
							Buf_UART4[2]=buffer[0];
							Buf_UART4[3]=21;
							Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
							Buf_UART4[5]=13;
							DMA_SetCurrDataCounter(DMA1_Stream4,6);
							DMA_Cmd(DMA1_Stream4,ENABLE);
						}
						break;
					case 5:
							if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
							{
									Buf_UART4[0]=13;
									Buf_UART4[1]=10;
									Buf_UART4[2]=buffer[0];
									Buf_UART4[3]=6;//ACK
									Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
									Buf_UART4[5]=13;
									DMA_SetCurrDataCounter(DMA1_Stream4,6);
									DMA_Cmd(DMA1_Stream4,ENABLE);
//									Press.Kp=buffer[1];
//									Press.Ki=buffer[2];
//									Press.Kd=buffer[3];
									Alt_PID.Kp=buffer[1];
									Alt_PID.Ki=buffer[2];
									Alt_PID.Kd=buffer[3];
//									Update_heso_Press=1; 
									Update_heso_Alt=1;	
										
									state_press=1;
									state_alt=0;
							}
							else
							{
									Buf_UART4[0]=13;
									Buf_UART4[1]=10;
									Buf_UART4[2]=buffer[0];
									Buf_UART4[3]=21;
									Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
									Buf_UART4[5]=13;
									DMA_SetCurrDataCounter(DMA1_Stream4,6);
									DMA_Cmd(DMA1_Stream4,ENABLE);
							}
							break;
						default:
							Buf_UART4[0]=13;
							Buf_UART4[1]=10;
							Buf_UART4[2]=buffer[0];
							Buf_UART4[3]=21;
							Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
							Buf_UART4[5]=13;
							DMA_SetCurrDataCounter(DMA1_Stream4,6);
							DMA_Cmd(DMA1_Stream4,ENABLE);
							break;
				}
			}		
		


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
// /*.................................ghi 9 chuoi...........................................*/			
// 			if(flag==1)
// 			{
// 				flag=0;
// 				lenght=y;
// 				if(lenght>100)
// 				{
// 					for(i=0;i<lenght;++i)
// 					{
// 						SD[i+lenght1]=Buf_USART2[i];
// 					}
// 					lenght1+=lenght;
//  					st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
// // 						if (!st_Result)
// // 						{
// // 							st_Result = f_lseek(&fil, f_size(&fil));
// // 							st_Result = f_write(&fil,SD,lenght1, bw);
// // 							st_Result = f_close(&fil); 
// // 						}
// // 						lenght1=0;
// 						flag2=1;
// 				}
// 				else if((lenght<100)&&(flag2==1))
// 				{
// 					flag3++;
// 					for(i=0;i<80;++i)
// 					{
// 						SD[i+lenght1]=Buf_USART2[i];
// 					}
// 					lenght1+=80;
// 					
// 					if(flag3==4)
// 					{
// 						flag3=0;
// 						flag2=0;
// 						//st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
// 						if (!st_Result)
// 						{
// 							st_Result = f_lseek(&fil, f_size(&fil));
// 							st_Result = f_write(&fil,SD,lenght1, bw);
// 							//st_Result = f_close(&fil); 
// 						}
// 						lenght1=0;
// 						flag4=1;
// 					}
// 				}
// 				else if((lenght<100)&&(flag4==1))
// 				{
// 					flag5++;
// 					for(i=0;i<80;++i)
// 					{
// 						SD[i+lenght1]=Buf_USART2[i];
// 					}
// 					lenght1+=80;
// 					 
// 				}
// 				if(flag5==4)
// 				{
// 					flag5=0;
// 					flag4=0;
// 					//st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
// 					if (!st_Result)
// 					{
// 						st_Result = f_lseek(&fil, f_size(&fil));
// 						st_Result = f_write(&fil,SD,lenght1, bw);
// 						st_Result = f_close(&fil);
// 					}
// 					lenght1=0;
// 				}
// 		}
	
/*.........................................................................................*/	

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
