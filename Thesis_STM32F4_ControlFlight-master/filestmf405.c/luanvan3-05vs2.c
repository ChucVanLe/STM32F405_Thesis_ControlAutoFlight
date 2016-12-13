/******************************************************************************
 *
 * Thesis of LeVanChuc
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

uint8_t index_find_e = 0;//variable for reset buffer receive data from GS

uint8_t Buf_rx4[1];

int i = 0;

/***********************************AnhHuan*******************************************/

char  data_IMU_GPS_CMD_tran_GS[500], Buf_USART2_trandata_to_GS[500];//data_IMU to 100ms tran data to GS
bool CMD_Trigger = false, CMD_Start_frame = false;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS
char data_from_pc[250];
extern bool control_path_use_stanley;//when receive data of path success--> control_path_use_stanley = true;


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
	//Configure PD2 read width pulse CH3
    UART4_Configuration(57600);//interface with GS
    USART2_Configuration(460800);//interface with GPS/IMU
    DMA_UART4_Configuration((uint8_t*)Buf_USART2_trandata_to_GS, 500);//receive data from IMU/GPS
    DMA_UART4_RX(Buf_rx4, 1);//receive data from GS

    MyTIM_PWM_Configuration();  
       
//    SysTick_Config(SystemCoreClock/500);//interrupt system 2ms
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
		//anh Huan_code GPS
		gps_init(460800);
    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))//auto control
					{//test ok:21/11/2016
						Call_Roll_PID(Roll_PID.SetPoint);   
						Call_Pitch_PID(Pitch_PID.SetPoint);
						Call_Yaw_PID(Yaw_PID.SetPoint);
						//real-time
						Call_Alt_PID(Alt_PID.SetPoint);		
        }

		//-----------------process data from IMU/GPS dataIMU 10ms, data GPS 100ms------------------------------------
		//-------------------get current value IMU/GPS----------------------------------
		//--------------------tran data to GS-------------------------------------------
		//---------------------receive data from GS-------------------------------------
		if(CMD_Trigger) //receive enough 1 frame command
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
//update code data IMU 10ms, data GPS 100ms	
//Anh Huan....................................................
		gps_process();//ok: get roll, pitch, yaw, lat, long, alt,
		if(control_path_use_stanley)
			main_control();//control flight use standley
			        
    }// end while
}//end main
    
/**************************************************************************************/





