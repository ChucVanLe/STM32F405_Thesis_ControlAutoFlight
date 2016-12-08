/******************************************************************************
 *                                                                            *
 *  @Project    SPRAYING MACHINE                                              *
 *  @Module     \MainBoard\MDL                                                *
 *	@File       gps.h                                                         *
 * 	@Author	    Dang Anh Tung                                                 *
 *  @History                                                                  *
 *              - Mar 23, 14: Initial version                                 *
 *                                                                            *
 ******************************************************************************/

#ifndef __GPS_H
#define __GPS_H
 
/******************************************************************************
 * 							        IMPORT                                    *
 ******************************************************************************/
#include "project.h"


/******************************************************************************
 * 							        DEFINE                                    *
 ******************************************************************************/
 
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
#define GPS_GPVTG_TRUE_HEADING_IND	1
#define GPS_GPVTG_MAG_HEADING_IND	3

#define IMU_ROLL_IND            1
#define IMU_PITCH_IND           2
#define IMU_YAW_IND             3
#define IMU_PRESS_IND           13

#define GPS_PROCESS_TIMEOUT     (uint32_t)500 //ms



 
/******************************************************************************
 * 							       TYPEDEF                                    *
 ******************************************************************************/


/******************************************************************************
 * 							GLOBAL VARIABLE                                   *
 ******************************************************************************/

extern GPS_STRUCT GPSStruct;
extern IMU_STRUCT IMUStruct;

/******************************************************************************
 * 							PRIVATE VARIABLE                                  *
 ******************************************************************************/
 

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

#endif /* __GPS_H */ 

