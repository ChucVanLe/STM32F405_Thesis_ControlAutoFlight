#include "luanvan.h"
#include "stdio.h"

	void Delay(void) 
{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
		{
   
    }
}
char buffer1[500],buffer2[500];
uint8_t data[500], data2[500],data1[500],data3[500];
uint8_t crc=0,crc1=0,crc2=0,luucrc=0,crclsb=0,crcmsb=0;
uint16_t a1=0, b=0x0107;
uint64_t delay,TickCnt;
int chieudai1,chieudai2,chieudai3=0,count1=0,count2=0,count3=0;
int i,y,k,k1,k2,i1,i2,i3,i4,l,h,j,j1,j2,x,flag,flag1,flag2,flag3,flag4;
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
int main (void)
{
	Delay(); 
	gpioconfigure ();
	USART2_configure (460800);
	USART3_configure (115200);
	DMA_UART2_RECIEVE(buffer1, 500);
	DMA_UART3_TRANSFER(buffer2,500);
  SysTick_Config(SystemCoreClock/1000);
	while(1)
 {
	 
 	 if(flag==1)
 	 {
		flag=0;
		count1=count2=500;
		if((k1==0)&&(k2==0))
			{
				if(y>90)
				{
				for(i=0;i<y;i++)
				{
					data[i]=buffer1[i];
				}
				DMA_UART2_RECIEVE(buffer1, 500);
				flag1=1;
			}
		else
					DMA_UART2_RECIEVE(buffer1, 500);
	}
		else if((k1==1)&&(k2==0))
			{
				j++;
				if(j==3)
				{
					j=0;
					flag3=1;
				}
				DMA_UART2_RECIEVE(buffer1, 500);
			}
		else if((k2==1)&&(k1==0))
			{
				j1++;
				if(j1==3)
				{
					j1=0;
					flag4=1;
				}
				DMA_UART2_RECIEVE(buffer1, 500);
			}
		else 
			{
				DMA_UART2_RECIEVE(buffer1, 500);
			}
		}
			
		
	 if(flag1==1)
	 {
		flag1=0;
		x=y;
		for(i=0;i<x;i++)
		{
			if(data[i]=='$')
			{
				k=1;
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
				
			 if(i2==1)
			  {
				 for(i=0;i<h-1;i++)
					{
					  data1[i]=data[i];
					}
					  chieudai1=h-1;
// 				  tinhcrc(data1,chieudai1);
// 					//data1[h-2]=0x20;
// 					data1[h-2]= luucrc ;
// 					data1[h-1]=13;
				 //GGA
				 for(i=h-1;i<l+1;i++)
					{
						data2[i-h+1]=data[i];
					}
					chieudai2 = l-h+2;
// 				tinhcrc(data2,chieudai2);
// 				//data2[l]=0x20;
// 				data2[l]= luucrc;
// 				data2[l+1]=13;
				 //VTG
				 for(i=l+1;i<x;i++)
					{
						data3[i-l-1]=data[i];
					}
					chieudai3 = x-l-1;
// 				tinhcrc(data3,chieudai3);
// 				//data3[x-1]=0x20;
// 				data3[x-1]= luucrc;
// 				data3[x]=13;
				 for(i=0;i<x;i++)
					{
						data[i]=0;
					}
					i2=0;
					i1=0;
					i3=0;
					GPIO_SetBits( GPIOD,GPIO_Pin_12);
					tinhcrc(data2,chieudai2);
					data2[l-h+1]=0x20;
					data2[l-h+2]= luucrc;
					data2[l-h+3]=13;
				  chieudai2=chieudai2 +2;
					while(k1==0)
					{		
						send(data2,chieudai2);

 						for(i=0;i<l-h+4;i++)
 						{
 					  
 						USART_SendData(USART3,(data2[i]&0xff));
 						while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
 						data2[i]=0;
 						}
						k1=1;
					}
						GPIO_ResetBits(GPIOD,  GPIO_Pin_12);
						k=0;
				 }
		  
		}
				
	if(flag3==1)
	{
	 k=1;
	 k1=0;
	 flag3=0;
	 GPIO_SetBits( GPIOD,GPIO_Pin_13);
	 tinhcrc(data3,chieudai3);
	 data3[x-l-2]=0x20;
	 data3[x-l-1]= luucrc;
	 data3[x-l]=13;
	 chieudai3=chieudai3+2;
 	 while(k2==0)
 	 {
		 send(data3,chieudai3);
 	 for(i=0;i<x-l+1;i++)
 		{
 		 USART_SendData(USART3,(data3[i]&0xff));
 		 while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
 		 data3[i]=0;
 		}
		 k2=1;
 	 }
		 GPIO_ResetBits(GPIOD,  GPIO_Pin_13);
	   k=0;
	 }
	
	
	if(flag4==1)
	{
		k=1;
		k2=0;
		flag4=0;
		GPIO_SetBits( GPIOD,GPIO_Pin_14);
		tinhcrc(data1,chieudai1);
		//data1[h-2]=0x20;
		data1[h-2]= luucrc ;
		data1[h-1]=13;
		chieudai1=chieudai1+1;
		while(k==1)
 		{
		 send(data1,h);
 		 for(i=0;i<h;i++)
 		  {
 		   USART_SendData(USART3,(data1[i]&0xff));
 		   while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET){}
 		   data1[i]=0;
 		  }
		   k=0;
	  }
		x=0;
		GPIO_ResetBits(GPIOD,  GPIO_Pin_14);
	}
 }
}

void SysTick_Handler();
