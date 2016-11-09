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

/**************************************************************************************/

#define		BUFF_SIZE			1//interrupt UART_DMA when receive BUFF_SIZE from GS
//#define		BUFF_SIZE_IMU_GPS			1024 
bool CMD_Trigger = false, timer_enough_2ms = false;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS
uint8_t Alt_latest = 0;
int compare_enough_data = 0;
int number_byte_empty_into_Buf_USART2 = 0;
uint8_t number_byte_empty_into_Buf_USART4_rx = 0, index_find_e = 0;//variable for receive from GS
int lenght_of_data_IMU_GPS = 0;
//char data_IMU[80], data_VTG[50], dataGGA[80];
uint8_t Buf_UART4[6], Buf_rx4[1];
char Buf_USART2[250],  data_from_pc[250];
uint8_t Update_heso_Roll=0,Update_heso_Pitch=0,Update_heso_Yaw=0,Update_heso_Alt=0,Update_heso_Press=0;

uint8_t index_receive_enough_data_GS = 0;//counter number of char from GS
uint16_t start_one_line_IMU_GPS, end_one_line_IMU_GPS;//find position start and end of 1 line into buffer IMU/GPS
int i = 0;
uint8_t find_char_$ = 0 ;
uint16_t position_VTG = 0, position_end_of_VTG = 0, position_end_of_GGA = 0;
uint8_t flag_set_or_current_press;
uint8_t state_alt = 1,state_press = 0;
void receive_data_and_reply(char *buffer);

float lat_long_from_GS[14][2];//contain lat, long which is uploaded from GS
float test_simulate1, test_simulate2, test_simulate3;
/***********************************AnhHuan*******************************************/

#define GPS_RX_BUF_LEN          (uint32_t)1000
#define GPS_SENTENCES           2

#define GPS_GPGGA_LAT_IND       2
#define GPS_GPGGA_LAT_HEM_IND   3
#define GPS_GPGGA_LON_IND       4
#define GPS_GPGGA_LON_HEM_IND   5
#define GPS_GPGGA_ALT_IND       9
#define GPS_GPGGA_ALT_HEM_IND   10
#define GPS_GPGGA_QA_IND        6
#define GPS_GPVTG_SPEED_IND     5 // knot

#define IMU_ROLL_IND            1
#define IMU_PITCH_IND           2
#define IMU_YAW_IND             3
#define GPS_RX_DMA_STREAM     DMA1_Stream5
 /* 							    GLOBAL VARIABLE                               */
/******************************************************************************/

GPS_STRUCT GPSStruct;
IMU_STRUCT IMUStruct;

#define GPS_PROCESS_TIMEOUT     (uint32_t)500 //ms
uint8_t GPSRxBuf[GPS_RX_BUF_LEN];
uint8_t GPSProcessTimeOut = 0;
uint32_t GPSReInd;
uint32_t GPSWrInd;
uint8_t data_IMU[100], GPSString[100], data_IMU_GPS_tran_GS[250];//data_IMU to 100ms tran data to GS

float gps_speed[10];
uint32_t tick1ms = 0;
uint8_t length_data_IMU_GPS_tran_GS = 0;
/******************************************************************************
 * 							PUBLIC FUNCTION                                   *
 ******************************************************************************/
void gps_init(uint32_t baudrate);
void gps_process(void);
 
/******************************************************************************
 * 							PRIVATE FUNCTION                                  *
 ******************************************************************************/
void gps_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd);
void imu_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd);
void gps_parse(uint8_t *gps_str);
void imu_parse(uint8_t *imu_str);
/******************************************************************************
 * @fn     GPS_DMA_ReadCount     
 * @brief  
 * @param  None
 * @retval None
 ******************************************************************************/
uint32_t GPS_DMA_ReadCount(void)
{
    return GPS_RX_DMA_STREAM->NDTR;
}
/******************************************************************************
 * @fn     SYSTIM_Tick     
 * @brief  Get the system tick (ms)
 *          This interface makes sure other functions can not change the system tick 
 * @param  
 * @retval None
 ******************************************************************************/
uint32_t SYSTIM_Tick(void)
{
    return tick1ms;
}
/******************************************************************************
 * @fn     SYSTIM_DelayTms     
 * @brief  Delay T(ms)
 * @param  T: delay time (ms)
 * @retval None
 ******************************************************************************/
BOOL SYSTIM_Timeout(uint32_t tim_tick, uint32_t time_out)
{
    uint32_t dt;
    uint32_t t = SYSTIM_Tick();
    if (tim_tick > t)
        dt = 0xFFFFFFFF - (tim_tick - t);
    else
        dt = t - tim_tick;
    
    if (dt >= time_out)
    {
        return TRUE;
    }
    return FALSE;
}
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
    DMA_UART4_Configuration(data_IMU_GPS_tran_GS,250);//tran data to GS
//    DMA_USART2_Config((uint8_t*)Buf_USART2, 250);//receive data from IMU/GPS
    DMA_UART4_RX(Buf_rx4, 1);//receive data from GS
// defaude dieu khien o che do ALT  
    state_press = 0;
    state_alt = 1;
    
    MyTIM_PWM_Configuration();  
       
    SysTick_Config(SystemCoreClock/500);//interrupt system 2ms
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
				receive_data_and_reply(&data_from_pc[0]);
				
				//reset data buffer
				CMD_Trigger = false;
				for(index_find_e = 0; index_find_e < 250; index_find_e ++)
						data_from_pc[index_find_e] = 0;
		}
		else//update code data IMU 10ms, data GPS 100ms	
			//Anh Huan....................................................
			gps_process();
			main_control();

//...........................	remove code save data because Mr.Huan has done it.				        
    }// end while
}//end main
    
/**************************************************************************************/
void SysTick_Handler(void)
{
	 timer_enough_2ms = true;
}

//Ngat nhan dmauart4
void DMA1_Stream2_IRQHandler(void)
{

		/* Clear the DMA1_Stream2 TCIF2 pending bit */
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);//DMA_IT_TCIF2 //co de bao day chua
		//format !data@
		if('!' == Buf_rx4[0])
			index_receive_enough_data_GS = 0;
		else index_receive_enough_data_GS++;
		
		data_from_pc[index_receive_enough_data_GS] = Buf_rx4[0];//copy du lieu ra, DMA_Mode_Circular: con tro tu quanh lai
		//mode nomal thi phai nop lai con tro
		//muon chay lan 2 thi enable lai
		//moi stream co 1 cai co, xoa co do thi moi bay DMA len duoc
		if('@' == Buf_rx4[0])//nhan duoc ky tu ket thuc chuoi bat dau xu ly
		{
			CMD_Trigger = true;
		}	
		/* reload new buff size for next reception */
		DMA1_Stream2->NDTR = BUFF_SIZE;
		DMA_Cmd(DMA1_Stream2, ENABLE);
}
void receive_data_and_reply(char *buffer)
{
	uint8_t i = 0, index_array_lat_lon = 0;
	uint8_t comma = 0;//find ','
	uint8_t fp_kp = 0, fp_ki = 0, fp_kd = 0, fp_setpoint = 0, ep_setpoint = 0, fp_lat = 0, fp_lon = 0;
	char temp_kp[10], temp_ki[10],temp_kd[10],temp_checksum[10],temp_setpoint[10], temp_lat[14], temp_long[14];
	uint32_t checksum_from_GS = 0, cal_check_sum = 0;
	//reset temp variable 
	for(i = 0; i < 10; i++)
	{
		temp_kp[i] = 0;
		temp_ki[i] = 0;
		temp_kd[i] = 0;
		temp_setpoint[i] = 0;
	}
	for(i = 0; i < 14; i++)
	{
		temp_lat[i] = 0;
		temp_long[i] = 0;
		lat_long_from_GS[i][0] = 0;
		lat_long_from_GS[i][1] = 0;
	}
	
	if('4' != buffer[1])//'4' = buffer[1]: upload latitude, longtitude
	{
		for (i=0; i <= index_receive_enough_data_GS; i++)
		{
			if (*(buffer + i)==',')
			{
				comma++;
				if(comma==1) fp_kp = i;
				if(comma==2) fp_ki = i;
				if(comma==3) fp_kd = i;
				if(comma==4) fp_setpoint = i;
				if(comma==5) ep_setpoint = i;
			}
		}
				//check sum
		for (i=1; i <= ep_setpoint; i++)
			{
				 cal_check_sum += *(buffer + i);
			}
	}

  switch (buffer[1])
				//'0': update roll
				//'1': update pitch
				//'2': update yaw
				//'3': update alt, state_alt = 1
				//'4': update lat, long
				//	update alt, state_alt = 0, no GPS, altitude is calculator with press, state press = 1
  {
  case '0':
	{

		//check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Roll_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Roll_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Roll_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Roll_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Roll=1;
			//send reply to GS
      data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 6;//ACK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 21;//NAK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}

    break;
	}
	case '1':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Pitch_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Pitch_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Pitch_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Pitch_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Pitch=1;
			//send reply to GS
      data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 6;//ACK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 21;//NAK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '2':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Yaw_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Yaw_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Yaw_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Yaw_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Yaw=1;
			//send reply to GS
      data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 6;//ACK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 21;//NAK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '3':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Alt_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Alt_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Alt_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Alt_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Alt=1;
			//send reply to GS
      data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 6;//ACK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 21;//NAK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '4'://update lat, long
	{//! 4 v lat k lon v lat k lon v lat k lon checksum @
		for (i=0; i <= index_receive_enough_data_GS; i++)
		{
			if(',' == *(buffer + i))//check sum
			{
				ep_setpoint = i;
				test_simulate1 = 1;
			}
		}		
		for (i=1; i <= ep_setpoint; i++)
		{
			 cal_check_sum += *(buffer + i);
		}
    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			//! 4 v lat k lon v lat k lon v lat k lon checksum @
			for (i=0; i <= index_receive_enough_data_GS; i++)
			{
				if ('k' == *(buffer + i)) 
				{
					fp_lon = i;			
					strncpy(temp_lat, &buffer[fp_lat + 1], fp_lon - fp_lat - 1);
					lat_long_from_GS[index_array_lat_lon][0] = atof(temp_lat);
				}
				else
				{
					if ('v' == *(buffer + i)) 
					{
						fp_lat = i;
						if(2 != fp_lat)
						{
						strncpy(temp_long, &buffer[fp_lon + 1], fp_lat - fp_lon - 1);
						lat_long_from_GS[index_array_lat_lon++][1] = atof(temp_long);
						}
					}
				}
			}
			//send respond to GS
      data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 6;//ACK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			data_IMU_GPS_tran_GS[0] = '!';
      data_IMU_GPS_tran_GS[1] = buffer[1];
      data_IMU_GPS_tran_GS[2] = 21;//NAK
      data_IMU_GPS_tran_GS[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}			

			break;
	}
                

//                        default:
//                            for ( i =0 ; i<9 ;++i )
//												    {
//                                *(buffer+i)=0;
//														}
//                            DMA_Cmd(DMA1_Stream2,DISABLE);
//                            DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
//                            DMA_SetCurrDataCounter(DMA1_Stream2,9);
//                            DMA_Cmd(DMA1_Stream2,ENABLE);                   
//                }
			}
}       
        


/**************************************************************************************/
/******************************************************************************
 * 							   PUBLIC FUNCTION                                *
 ******************************************************************************/
 
/******************************************************************************
 * @fn     gps_init     
 * @brief  
 * @retval None
 ******************************************************************************/
void gps_init(uint32_t baudrate)
{
    uint32_t i;
    
    if (baudrate != 0) USART2_Configuration(baudrate);

    GPSStruct.isavailable = FALSE;
    GPSStruct.quality = GPS_QA_NOT_FIX;
    GPSStruct.latitude = 0;
    GPSStruct.longitude = 0;
    GPSStruct.speed = 0;
    
    for (i = 0; i < GPS_RX_BUF_LEN; GPSRxBuf[i] = 0, i++);
    //GPS_DMA_Read(&GPSRxBuf[0], GPS_RX_BUF_LEN);
	  DMA_USART2_Config((uint8_t*)&GPSRxBuf[0], GPS_RX_BUF_LEN);//receive data from IMU/GPS
    GPSReInd = 0;
    GPSWrInd = 0;
    
    for (i = 0; i < 10; gps_speed[i] = 0, i++);
}


/******************************************************************************
 * @fn     gps_process     
 * @brief  
 * @retval 
 ******************************************************************************/
void gps_process(void)
{
    static uint32_t gps_time_tick;
    static uint32_t GPSWrInd_1 = 0;
    static uint32_t start_ind = 0, end_ind = 0, state = 0;
    uint32_t i, j, k, len, len1;

    // Calculate a number of reception bytes
    GPSWrInd = GPS_RX_BUF_LEN - GPS_DMA_ReadCount();
    
    if (GPSWrInd_1 != GPSWrInd)
    {
        gps_time_tick = SYSTIM_Tick();
        GPSWrInd_1 = GPSWrInd;
    }
    else if (SYSTIM_Timeout(gps_time_tick, GPS_PROCESS_TIMEOUT))
    {
        // reset
        GPSStruct.isavailable = FALSE;
        GPSReInd = GPSWrInd;
        state = 0;
        for (i = 0; i < 10; gps_speed[i] = 0, i++);
    }
    
    // Length of reception string
    if (GPSReInd <= GPSWrInd) len = GPSWrInd - GPSReInd;
    else len = GPS_RX_BUF_LEN - (GPSReInd - GPSWrInd);
    
    // Scan through this string
    // Note: i++ will increase variable i after one loop
    for (i = 0; i < len; i++)
    {
        j = (GPSReInd + i) % GPS_RX_BUF_LEN;
        switch (state)
        {
            case 0: // wait for start_ind
                if (GPSRxBuf[j] == '\n')
                {
                    start_ind = j;
                    state = 1;
                }
                break;
            case 1: // wait for end_ind
                if (GPSRxBuf[j] == '\r')
                {
                    end_ind = j;
                    state = 0;
                    // Copy string from start_ind to end_ind
                    if (start_ind < end_ind) len1 = end_ind - start_ind + 1;
                    else len1 = GPS_RX_BUF_LEN - (start_ind - end_ind) + 1;
										if(' ' == GPSRxBuf[(start_ind + 2)%GPS_RX_BUF_LEN])//data IMU
										{
											length_data_IMU_GPS_tran_GS = len1;
											for (k = 0; k < len1; k++)
											{
												data_IMU[k] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												data_IMU_GPS_tran_GS[k] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
											}
											imu_parse(&data_IMU[0]);
										}
										else
										{
											if('V' == GPSRxBuf[(start_ind + 4)%GPS_RX_BUF_LEN])//data GPS_VTG
											{
												for (k = 0; k < len1; k++)
												{
                        GPSString[k] = GPSRxBuf[(start_ind + k)%GPS_RX_BUF_LEN];//remove '\n'
												data_IMU_GPS_tran_GS[k + length_data_IMU_GPS_tran_GS] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												}
											gps_parse(&GPSString[1]);
											length_data_IMU_GPS_tran_GS += len1;

											}
											if('G' == GPSRxBuf[(start_ind + 4)%GPS_RX_BUF_LEN])//data GPS_GGA
											{
												for (k = 0; k < len1; k++)
												{
                        GPSString[k] = GPSRxBuf[(start_ind + k)%GPS_RX_BUF_LEN];//remove '\n'
												data_IMU_GPS_tran_GS[k + length_data_IMU_GPS_tran_GS] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												}
											gps_parse(&GPSString[1]);
											//tran data to GS
											DMA_SetCurrDataCounter(DMA1_Stream4, length_data_IMU_GPS_tran_GS + len1);
											DMA_Cmd(DMA1_Stream4,ENABLE);
											}
										}
                    //DIS_DMA_Write(&GPSString[0], len1); //for debug only
                    // Process string
//                    if (GPSString[2] == 'G')
//                    {
//                        //strcpy(GPSString, "$GPVTG,37.313,T,37.313,M,0.061,N,0.113,K*4A G\n");
//                        //strcpy(GPSString, "$GPGGA,084808.00,1046.3727894,N,10639.5866087,E,4,05,1.9,2.252,M,0.00,M,01,0000*66 G\n");
//                        //strcpy(GPSString, "$GPGGA,084808.00,1046.37341245,N,10639.58681519,E,4,09,1.2,11.529,M,2.291,M,1.0,0677*40 R\n");
//                        gps_parse(&GPSString[0]);
//											//tran data to GS
//											DMA_SetCurrDataCounter(DMA1_Stream4, lenght_of_data_IMU_GPS);
//											DMA_Cmd(DMA1_Stream4,ENABLE);  
//                    }
//                    else if (GPSString[2] == ' ')
//                    {
//                        //$  0010 -0010  0364  0007  0006  0020  0029  0013  1055  0463  0673  0103
//                        imu_parse(&GPSString[0]);
//                    }
                }
                break;
            default:
                state = 0;
                break;
        }
    }
    
    GPSReInd = GPSWrInd;

}


/******************************************************************************
 * 							PRIVATE FUNCTION                                  *
 ******************************************************************************/

/******************************************************************************
 * @fn     gps_getfieldind     
 * @brief  
 * @retval 
 ******************************************************************************/
void gps_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd)
{
    uint8_t i = startInd;
  
    *fieldInd++ = i;
    while (buf[i] != '\n')
    {
        if (buf[i] == ',')
        {
            *fieldInd++ = (i + 1);
        }
        i++;
    }
}

/******************************************************************************
 * @fn     gps_parse     
 * @brief  
 * @retval 
 ******************************************************************************/
void gps_parse(uint8_t *gps_str)
{
    static uint8_t field_ind[20];
    uint32_t ind0, ind1, len;
    char *p; // cast to this pointer to avoid warning from compiler for atoi function

    if ((gps_str[3] == 'V') &&
        (gps_str[4] == 'T') &&
        (gps_str[5] == 'G'))
    {
        GPSStruct.isavailable = TRUE;
        gps_getfieldind(gps_str, 0, field_ind);
        
        // speed
        ind0 = field_ind[GPS_GPVTG_SPEED_IND];
        ind1 = field_ind[GPS_GPVTG_SPEED_IND + 1];
        len = ind1 - ind0;
        if (len != 1)
        {
            p = (char*)&gps_str[ind0];
            GPSStruct.speed = atof(p);
            GPSStruct.speed *= 1.852f; // knot to km/h 
        }
        else
        {
            GPSStruct.speed = 0;
            GPSStruct.quality = GPS_QA_NOT_FIX;
        }
    }
    else if ((gps_str[3] == 'G') &&
             (gps_str[4] == 'G') &&
             (gps_str[5] == 'A'))
    {
        GPSStruct.isavailable = TRUE;
        gps_getfieldind(gps_str, 0, field_ind);

        // quality
        ind0 = field_ind[GPS_GPGGA_QA_IND];
        if ((gps_str[ind0] >= '0') && 
            (gps_str[ind0] <= '9'))
        {
            GPSStruct.quality = (GPS_QA)(gps_str[ind0] - '0');
        }
        else
        {
            GPSStruct.quality = GPS_QA_NOT_FIX;
        }

        // Position
        if (GPSStruct.quality != GPS_QA_NOT_FIX)
        {
            // latitude
            ind0 = field_ind[GPS_GPGGA_LAT_IND];
            ind1 = field_ind[GPS_GPGGA_LAT_IND + 1];
            len = ind1 - ind0;
            if (len > 7)
            {
                p = (char*)&gps_str[ind0]; //pointer to string
                GPSStruct.latitude = (p[0] - '0') * 10.0f + (p[1] - '0') * 1.0f;
                GPSStruct.latitude += atof(&p[2]) / 60.0f;
                ind0 = field_ind[GPS_GPGGA_LAT_HEM_IND];
                if (gps_str[ind0] == 'W') { GPSStruct.latitude = -GPSStruct.latitude;}
            }
            
            // longitude
            ind0 = field_ind[GPS_GPGGA_LON_IND];
            ind1 = field_ind[GPS_GPGGA_LON_IND + 1];
            len = ind1 - ind0;
            if (len > 7)
            {
                p = (char*)&gps_str[ind0];
                GPSStruct.longitude = (p[0] - '0') * 100.0f   + (p[1] - '0') * 10.0f + (p[2] - '0') * 1.0f;
                GPSStruct.longitude += atof(&p[3]) / 60.0f;
                ind0 = field_ind[GPS_GPGGA_LON_HEM_IND];
                if (gps_str[ind0] == 'S') {GPSStruct.longitude = -GPSStruct.longitude;}
                
                GPSStruct.isready = TRUE;
            }
						
						// alt
            ind0 = field_ind[GPS_GPGGA_ALT_IND];
						ind1 = field_ind[GPS_GPGGA_ALT_IND + 1];
            len = ind1 - ind0;

						if (len != 1)
						{
						p = (char*)&gps_str[ind0];
            GPSStruct.altitude = atof(p);
						}
						else
						{
								GPSStruct.altitude = 0;
						}
						Alt_PID.Current = GPSStruct.altitude;
						Alt_PID.Enable = 1 ;
        }
    }
    
}

/******************************************************************************
 * @fn     imu_parse:  
 * @brief  "$ roll pitch yaw"
 * @retval 
 ******************************************************************************/
void imu_parse(uint8_t *imu_str)
{
    static uint8_t field_ind[20];
    char *p; // cast to this pointer to avoid warning from compiler for atoi function

    IMUStruct.isavailable = TRUE;
    imu_getfieldind(imu_str, 0, field_ind);
   
    //Roll angle
    p = (char*)&imu_str[field_ind[IMU_ROLL_IND]];
    IMUStruct.roll = atoi (p) * 0.1f;
   
    // Pitch angle
    p = (char*)&imu_str[field_ind[IMU_PITCH_IND]];
    IMUStruct.pitch = atoi (p) * 0.1f;
    
    // yaw angle
    p = (char*)&imu_str[field_ind[IMU_YAW_IND]];
    IMUStruct.yaw = atoi (p) * 0.1f;
	//update roll, pitch, yaw
		Roll_PID.Current = IMUStruct.roll;
		Roll_PID.Enable = 1;
		Pitch_PID.Current = IMUStruct.pitch;
		Pitch_PID.Enable = 1;
		Yaw_PID.Current = IMUStruct.yaw;
		Yaw_PID.Enable = 1;
}

/******************************************************************************
 * @fn     gps_getfieldind     
 * @brief  
 * @retval 
 ******************************************************************************/
void imu_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd)
{
    uint8_t i = startInd;

    *fieldInd++ = i;
    while (buf[i] != '\r')
    {
        if ((buf[i] == '-') || 
            ((buf[i] >= '0') &&
             (buf[i] <= '9')))
        {
            *fieldInd++ = i;
            i += 4;
        }
        i++;
    }
}

