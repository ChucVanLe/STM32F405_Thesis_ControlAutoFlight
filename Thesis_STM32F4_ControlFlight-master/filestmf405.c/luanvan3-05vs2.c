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

int32_t clk_1microsec = 0;
/***********************************AnhHuan*******************************************/


bool CMD_Trigger = false;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS

extern bool control_path_use_stanley;//when receive data of path success--> control_path_use_stanley = true;


/******************************************************************************
 * 							PUBLIC FUNCTION                                   *
 ******************************************************************************/

/*************************************************************************************/
int main(void)
{
		config_gpio_and_interrupt();
		config_uart_2_4();
		SysTick_Config(SystemCoreClock/1000000);//interrupt system 1 microsec
	
    while(1)
    {
//        if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))//auto control
//					{//test ok:21/11/2016
//						Call_Roll_PID(Roll_PID.SetPoint);   
//						Call_Pitch_PID(Pitch_PID.SetPoint);
//						Call_Yaw_PID(Yaw_PID.SetPoint);
//						//real-time
//						Call_Alt_PID(Alt_PID.SetPoint);		
//        }
			//--------------------tran data to GS-------------------------------------------
			//---------------------receive data from GS-------------------------------------
			if(CMD_Trigger) //receive enough 1 frame command
			{//if CMD_Trigger = 1 ; receive data from ground station 
				reset_buffer_data_receive_data();
			}
			
			//-----------------process data from IMU/GPS dataIMU 10ms, data GPS 100ms------------------------------------
			//-------------------get current value IMU/GPS----------------------------------
			//Anh Huan....................................................
			gps_process();//ok: get roll, pitch, yaw, lat, long, alt,
			if(control_path_use_stanley)
				main_control();//control flight use standley

			        
    }// end while
}//end main
    
/**************************************************************************************/
/**************************************************************************************/
void SysTick_Handler(void)
 {
     clk_1microsec++;
 }




