#include "stm32f4xx.h"
#include "stdio.h"
/*/////                          vai ham config can thiet  	                    /////////////////
	1. Delay() : delay truoc chuong trinh
	2. uint8_t tinhcrc (uint8_t offsetmin,uint8_t *kitu,int offsetmax)
								kitu : mang [0...
								offsetmin : vitri bat dau
								offsetmax : vitri ket thuc
	3. gpioconfigure () : cau hinh chan
	4. timer()                    	: cau hinh TIM2 chu ki 50ms couter xung 1Mhz, dem len
	5.USART2_configure () 					 : cau hinh USART2  truyen nhan voi IMU GPS  
								PA2 : TX 
								PA3:	RX
	6.USART3_configure ()
								PB10:TX
								pB11:RX
	
*******************************************************************************************************/


/***********************************************************************************************************/

void Delay(void) 
		{
    uint32_t m;
    for(m=0; m<0xffffff; ++m) 
			{
			}
		}
/***********************************************************************************************************/
uint8_t tinhcrc (uint8_t offsetmin,uint8_t *kitu,int offsetmax)
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
/***********************************************************************************************************/

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
		//cau hinh GPIO uart3
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
		TIM_TimeBaseStructure.TIM_Period = 50000-1;  //50ms
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, DISABLE);
  }
	/***********************************************************************************************************/

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
/***********************************************************************************************************/
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
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

