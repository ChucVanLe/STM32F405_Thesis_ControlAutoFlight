/******************************************************************************
 *
 * thesis of LeVanChuc
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Main controller of system
 *
 ******************************************************************************/

/******************************************************************************
 *
 * 	Module       : luanvan3-05vs2.c
 * 	Description  : This is code for STM32F405 to receive data from IMU/GPS, receive command from ground station
 *									process data, control flight in auto mode across FPGA
 *
 *  Tool           : keilC 4
 *  Chip           : STM32F405
 *  History        : 27-08-2016
 *  Version        : 1.0
 *
 *  Author         : Le Van Chuc, CLB NCKH DDT (levanchuc94qn@gmail.com)
 *  Notes          :
 *
******************************************************************************/
#include "project.h"

/**************************************************************************************/
 /* 							    GLOBAL VARIABLE                               */
/******************************************************************************/

uint8_t index_find_e = 0;//variable for receive from GS
int lenght_of_data_IMU_GPS = 0;
uint8_t Buf_rx4[1];
uint8_t Update_heso_Roll=0,Update_heso_Pitch=0,Update_heso_Yaw=0,Update_heso_Alt=0,Update_heso_Press=0;
int i = 0;
uint8_t state_alt = 1,state_press = 0;
float test_simulate1, test_simulate2, test_simulate3;
/***********************************AnhHuan*******************************************/

char  data_IMU_GPS_CMD_tran_GS[500], Buf_USART2_trandata_to_GS[500];//data_IMU to 100ms tran data to GS
bool CMD_Trigger = false, CMD_Start_frame = false;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS
char data_from_pc[250];


/******************************************************************************
 * 							PUBLIC FUNCTION                                   *
 ******************************************************************************/
 


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
    UART4_Configuration(57600);
    USART2_Configuration(460800);
//    DMA_UART4_Configuration((uint8_t*)data_IMU_GPS_CMD_tran_GS,250);//tran data to GS
    DMA_UART4_Configuration((uint8_t*)Buf_USART2_trandata_to_GS, 500);//receive data from IMU/GPS
    DMA_UART4_RX(Buf_rx4, 1);//receive data from GS
// defaude dieu khien o che do ALT  
    state_press = 0;
    state_alt = 1;
    
    MyTIM_PWM_Configuration();  
       
//    SysTick_Config(SystemCoreClock/500);//interrupt system 2ms
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
		//anh Huan_code GPS
		gps_init(460800);
    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))//auto
        {
						Call_Roll_PID(Roll_PID.SetPoint);   
						Call_Pitch_PID(Pitch_PID.SetPoint);
						Call_Yaw_PID(Yaw_PID.SetPoint);
						//test simute altitude
//						if(Yaw_PID.Current > 0)
//						Alt_PID.Current = Yaw_PID.Current;
//						else Alt_PID.Current = 0;
//						Call_Alt_PID(30);
						//real-time
						Call_Alt_PID(Alt_PID.SetPoint);		
        }

		//-----------------process data from IMU/GPS dataIMU 10ms, data GPS 100ms------------------------------------
		//-------------------get current value IMU/GPS----------------------------------
		//--------------------tran data to GS-------------------------------------------
		//---------------------receive data from GS-------------------------------------
		if(CMD_Trigger) //receive enough 1 frame
		{//if CMD_Trigger = 1 ; receive data from ground station 
				Delay_100ms();
				receive_data_and_reply(&data_from_pc[0]);
			  Delay_100ms();

				//reset data buffer
				CMD_Trigger = false;
				CMD_Start_frame = false;
				for(index_find_e = 0; index_find_e < 250; index_find_e ++)
						data_from_pc[index_find_e] = 0;

		}
//		else//update code data IMU 10ms, data GPS 100ms	
			//Anh Huan....................................................
			gps_process();//ok
			//main_control();

//...........................	remove code save data because Mr.Huan has done it.				        
    }// end while
}//end main
    
/**************************************************************************************/





