#include "CONNECTWITHGS.h"
#include "project.h"
//------------------------------
//variable
double utm_lat_long_from_GS[14][2];//contain lat, long which is uploaded from GS
uint8_t index_receive_enough_data_GS = 0;//counter number of char from GS
extern char  data_IMU_GPS_CMD_tran_GS[500];//data_IMU to 100ms tran data to GS
uint8_t number_byte_respone_to_GS = 0;
extern bool CMD_Trigger, CMD_Start_frame;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS
extern char data_from_pc[250];
bool control_path_use_stanley = false, switch_to_control_flight_use_standley = false;
#define		BUFF_SIZE			1//interrupt UART_DMA when receive BUFF_SIZE from GS
//------------------------------
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
		utm_lat_long_from_GS[i][0] = 0;
		utm_lat_long_from_GS[i][1] = 0;
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
			//Update_heso_Roll=1;
			//send reply to GS
      data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 6;//ACK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
//			DMA_SetCurrDataCounter(DMA1_Stream4,40);
//      
		}
		else
		{
			data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 21;//NAK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
//			number_byte_respone_to_GS = 4;
//     
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
			//Update_heso_Pitch=1;
			//send reply to GS
      data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 6;//ACK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
		}
		else
		{
			data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 21;//NAK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
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
			//Update_heso_Yaw=1;
			//send reply to GS
      data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 6;//ACK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
		}
		else
		{
			data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 21;//NAK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
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
			//Update_heso_Alt=1;
			//send reply to GS
      data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 6;//ACK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
		}
		else
		{
			data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 21;//NAK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
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
					utm_lat_long_from_GS[index_array_lat_lon][0] = atof(temp_lat);
				}
				else
				{
					if ('v' == *(buffer + i)) 
					{
						fp_lat = i;
						if(2 != fp_lat)
						{
						strncpy(temp_long, &buffer[fp_lon + 1], fp_lat - fp_lon - 1);
						utm_lat_long_from_GS[index_array_lat_lon++][1] = atof(temp_long);
						}
					}
				}
			}
			//send respond to GS
      data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 6;//ACK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      control_path_use_stanley = true;
			switch_to_control_flight_use_standley = true;
		}
		else
		{
			data_IMU_GPS_CMD_tran_GS[0] = '!';
      data_IMU_GPS_CMD_tran_GS[1] = buffer[1];
      data_IMU_GPS_CMD_tran_GS[2] = 21;//NAK
      data_IMU_GPS_CMD_tran_GS[3] = '@';
			number_byte_respone_to_GS = 4;
      
		}			

			break;
		}             
	}
}       
//--------------------------------------------------------       
//Ngat nhan dmauart4
void DMA1_Stream2_IRQHandler(void)
{

		/* Clear the DMA1_Stream2 TCIF2 pending bit */
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);//DMA_IT_TCIF2 //co de bao day chua
		//format !data@
		if('!' == Buf_rx4[0])
			{
			index_receive_enough_data_GS = 0;
			CMD_Start_frame = true;
			}

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

/**************************************************************************************/
