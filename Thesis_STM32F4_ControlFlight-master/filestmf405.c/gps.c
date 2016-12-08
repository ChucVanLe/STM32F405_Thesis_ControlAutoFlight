/******************************************************************************
 *                                                                            *
 *  @Project    SPRAYING MACHINE                                              *
 *  @Module     \MainBoard\MDL                                                *
 *	@File       gps.c                                                        *
 * 	@Author	    Dang Anh Tung                                                 *
 *  @History                                                                  *
 *              - Mar 23, 14: Initial version                                 *
 *                                                                            *
 ******************************************************************************/
 

/******************************************************************************
 * 							        IMPORT                                    *
 ******************************************************************************/
#include "gps.h"
#include <stdlib.h> 
/******************************************************************************
 * 							        DEFINE                                    *
 ******************************************************************************/


 
/******************************************************************************
 * 							    GLOBAL VARIABLE                               *
 ******************************************************************************/
GPS_STRUCT GPSStruct;
IMU_STRUCT IMUStruct;

/******************************************************************************
 * 							   PRIVATE VARIABLE                               *
 ******************************************************************************/
uint8_t GPSRxBuf[GPS_RX_BUF_LEN];
uint8_t GPSProcessTimeOut = 0;
uint32_t GPSReInd;
uint32_t GPSWrInd;
uint8_t GPSString[100];
real32_t gps_speed[10];
uint8_t data_IMU[85];

GPS_STRUCT GPSStruct;
IMU_STRUCT IMUStruct;

#define GPS_PROCESS_TIMEOUT     (uint32_t)500 //ms

extern char  data_IMU_GPS_CMD_tran_GS[500], Buf_USART2_trandata_to_GS[500];//data_IMU to 100ms tran data to GS
extern uint8_t number_byte_respone_to_GS;
float gps_speed[10];

uint8_t length_data_IMU_GPS_CMD_tran_GS = 0;
#define GPS_RX_DMA_STREAM     DMA1_Stream5
uint32_t tick1ms = 0;
extern bool CMD_Start_frame;
/******************************************************************************
 * 							   PUBLIC FUNCTION                                *
 ******************************************************************************/
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
											length_data_IMU_GPS_CMD_tran_GS = len1;
											for (k = 0; k < len1; k++)
											{
												data_IMU[k] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												data_IMU_GPS_CMD_tran_GS[number_byte_respone_to_GS + k] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
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
												data_IMU_GPS_CMD_tran_GS[number_byte_respone_to_GS + k + length_data_IMU_GPS_CMD_tran_GS] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												}
											gps_parse(&GPSString[1]);
											length_data_IMU_GPS_CMD_tran_GS += len1;

											}
											if('G' == GPSRxBuf[(start_ind + 4)%GPS_RX_BUF_LEN])//data GPS_GGA
											{
												for (k = 0; k < len1; k++)
												{
                        GPSString[k] = GPSRxBuf[(start_ind + k)%GPS_RX_BUF_LEN];//remove '\n'
												data_IMU_GPS_CMD_tran_GS[number_byte_respone_to_GS + k + length_data_IMU_GPS_CMD_tran_GS] = GPSRxBuf[(start_ind + k)% GPS_RX_BUF_LEN];//remove '\n'											
												}
												gps_parse(&GPSString[1]);
												//tran data to GS
												if(!CMD_Start_frame)//only tran or receive data
													{
													strcpy(Buf_USART2_trandata_to_GS, data_IMU_GPS_CMD_tran_GS);
													DMA_SetCurrDataCounter(DMA1_Stream4, number_byte_respone_to_GS + length_data_IMU_GPS_CMD_tran_GS + len1);
													DMA_Cmd(DMA1_Stream4,ENABLE);
													number_byte_respone_to_GS = 0;
													}
											}
										}
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
				// heading
        ind0 = field_ind[GPS_GPVTG_TRUE_HEADING_IND];
        ind1 = field_ind[GPS_GPVTG_TRUE_HEADING_IND + 1];
        len = ind1 - ind0;
        if (len != 1)
        {
            p = (char*)&gps_str[ind0];
            GPSStruct.heading = atof(p);
        }
        else
        {
            GPSStruct.heading = 0;
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
		double temp_alt;
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
	  // press value
    p = (char*)&imu_str[field_ind[IMU_PRESS_IND]];
    IMUStruct.press = atoi (p) * 0.1f;
		//calculate alt
		temp_alt = 1127;
		temp_alt += 100000;
		temp_alt = 1- pow((temp_alt/101325),(1/5.25578));
		temp_alt = (temp_alt*pow(10,5))/2.25577;
		IMUStruct.alt_press = temp_alt;
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
