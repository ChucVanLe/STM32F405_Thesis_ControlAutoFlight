/******************************************************************************
 *                                                                            *
 *  @Project    SPRAYING MACHINE                                              *
 *  @Module     \MainBoard\COMMON                                             *
 *	@File       mainboard.c                                                   *
 * 	@Author	    Dang Anh Tung                                                 *
 *  @History                                                                  *
 *              - Mar 09, 14: Initial version                                 *
 *                                                                            *
 ******************************************************************************/
 #include "project.h"
#ifndef __DEF_H
#define __DEF_H

/*******************************************************************************
 * COMMON DEFINITIONS
 ******************************************************************************/

#define NULL    0


typedef enum
{
    BIT0 = (uint8_t)0,
    BIT1 = (uint8_t)1
}BIT;

/*******************************************************************************
 * NETWORK MASTER/SLAVE DEFINITIONS
 ******************************************************************************/
typedef struct
{
    uint32_t id; // message id
    uint8_t* data; // the length of data may be changed
}NET_MSG;

typedef struct
{
    BOOL isvalid;
    uint8_t data[8];
}SLAVE_ARRAY;

typedef struct 
{
    int16_t  speed_set; // set speed from MASTER
    int16_t  speed_cur; // current speed on SLAVE
    uint16_t valve_set; // set output signals from MASTER
    uint16_t input_cur; // current input signals on SLAVE
    uint32_t time_tick; // how long did the SLAVE does not send a message to MASTER
    BOOL     isvalid;   // TRUE if this SLAVE is connected to the system
    uint8_t  id;        // board/slave id
}SLAVE_STRUCT;

typedef enum
{
    SLAVE_CMD_TOTAL   = (uint8_t)0, /* Not used*/
    SLAVE_CMD_SPEED   = (uint8_t)1, /* Data: |Speed_Set(2bytes)|0x00,...| */
    SLAVE_CMD_INPUT   = (uint8_t)2, /* Not used. */
    SLAVE_CMD_VALVE   = (uint8_t)3, /* Data: |Valve_Set(2bytes)|0x00,...| */
    SLAVE_CMD_KP      = (uint8_t)4, /* Data: |Kp(4bytes)|.... */
    SLAVE_CMD_KI      = (uint8_t)5, /* Data: |Ki(4bytes)|.... */
    SLAVE_CMD_KD      = (uint8_t)6, /* Data: |Kd(4bytes)|....  */
    SLAVE_CMD_ENC_RES = (uint8_t)7, /* Data: |res(2bytes)|.... */ 
    SLAVE_CMD_RUN_EN  = (uint8_t)8, /* Data: |en(1byte)|... */
}SLAVE_CMD;


typedef enum
{
    SLAVE_CONNECT    = (uint8_t)0,
    SLAVE_DISCONNECT = (uint8_t)1
}SLAVE_EVENT;

typedef void (*VOID_PFUNC_VOID)(void); 
typedef void (*VOID_PFUNC_SLAVE_EV)(uint8_t id, SLAVE_EVENT e);

/*******************************************************************************
 * FRAME DISPLAY BOARD DEFINITIONS
 ******************************************************************************/
typedef enum
{
    FRAME_WAITING_HEADER = (uint8_t)0,
    FRAME_WAITING_DATA = (uint8_t)1,
    FRAME_COMPLETE = (uint8_t)2,
}FRAME_STATE;

typedef enum
{
    CMD_NULL = (uint8_t)0,
    CMD_001  = (uint8_t)1,
    CMD_002  = (uint8_t)2,
    CMD_003  = (uint8_t)3,
    CMD_004  = (uint8_t)4,
    
    CMD_011  = (uint8_t)11,
    CMD_012  = (uint8_t)12,
    CMD_013  = (uint8_t)13,
    CMD_014  = (uint8_t)14,
    CMD_015  = (uint8_t)15,
    
    CMD_021  = (uint8_t)21,
    CMD_022  = (uint8_t)22,
    
    CMD_101  = (uint8_t)101,
    CMD_102  = (uint8_t)102,
    
    CMD_200  = (uint8_t)200,
    CMD_201  = (uint8_t)201,
    CMD_202  = (uint8_t)202,
    CMD_250  = (uint8_t)250,
}FRAME_CMD;

typedef struct
{
    uint32_t    time_tick;
    FRAME_STATE state;
    uint8_t     len;
    FRAME_CMD   cmd;
    uint8_t     *data;
}FRAME_RXBUF;

typedef struct
{
    BOOL        isvalid;
    uint8_t     len;
    FRAME_CMD   cmd;
    uint8_t     data[200];
}FRAME_STRUCT;


typedef enum
{
    FRAME_RES_OK,
    FRAME_RES_FAIL
}FRAME_RES;


typedef float real32_t;
typedef double real64_t;

typedef enum
{
    GPS_QA_NOT_FIX   = (uint8_t)0,
    GPS_QA_FIX       = (uint8_t)1,
    GPS_QA_D_FIX     = (uint8_t)2,
    GPS_QA_RTK_INT   = (uint8_t)4,
    GPS_QA_RTK_FLOAT = (uint8_t)5,
    GPS_QA_EST       = (uint8_t)6
}GPS_QA;


typedef struct 
{
    BOOL isavailable; //True: received GPS string
    BOOL isready; // True: received new valid gps tring, used as trigger 100ms for controller
    GPS_QA quality; // 4: RTK fixed
    real64_t latitude;
    real64_t longitude;
		real64_t altitude;
    real32_t speed;
    real32_t heading;
		real32_t mag_heading;
		real32_t alt_press;
}GPS_STRUCT;

typedef struct
{
    BOOL isavailable;
    real32_t roll;
    real32_t pitch;
    real32_t yaw;
		real32_t press;
		real32_t alt_press;
}IMU_STRUCT;

typedef struct
{
	real64_t x;
	real64_t y;
	real64_t R1;
	real64_t R2;	
	real64_t WPangle;
}WAYPOINT_STRUCT;

/*******************************************************************************
 * MEMORY ADDRESS
 ******************************************************************************/
typedef enum
{
    MEM_PRESSURE_RANGE = (uint16_t)0,  /* Pressure range (3psi): 2 bytes, int16_t */
    MEM_GPS_BAUD       = (uint16_t)10, /* GPS baudrate (96): 2 bytes, int16_t, /100 */
    MEM_RS485_BAUD     = (uint16_t)20, /* RS485 baudrate (96): 2 bytes, int16_t, /100 */
    MEM_ISO_BAUD       = (uint16_t)30, /* ISO Bus baudrate: reserved */
    MEM_MASTER_PID     = (uint16_t)40,  /* Master PID parameters (0.9, 1.1, 0): 12 bytes, Kp, Ki, Kd, float */
    MEM_SLAVE_PID      = (uint16_t)60,  /* Slave PID parameters (0.9, 1.1, 0): 12 bytes, Kp, Ki, Kd, float */
    MEM_ENC_RES        = (uint16_t)80   /* Encoder resolution (400ppr):2 bytes, int16_t */
}MEM_ADD;

#endif /* __DEF_H */
