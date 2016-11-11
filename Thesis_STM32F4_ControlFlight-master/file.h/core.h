/******************************************************************************
 *                                                                            *
 *  @Project    SPRAYING MACHINE                                              *
 *  @Module     \ControlBoard\MDL                                             *
 *	@File       core.h                                                        *
 * 	@Author	    Dang Anh Tung                                                 *
 *  @History                                                                  *
 *              - Mar 27, 14: Initial version                                 *
 *                                                                            *
 ******************************************************************************/

#ifndef __CORE_H
#define __CORE_H
 
/******************************************************************************
 * 							        IMPORT                                    *
 ******************************************************************************/
//#include "mainboard.h"
//#include "gps.h"
#include "project.h"
#include "def.h"
#include "LL2UTM.h"


/******************************************************************************
 * 							        DEFINE                                    *
 ******************************************************************************/
/* CONTROL PARAMS */
#define STEER_CONTROL_TIME (uint32_t)5   // ms
#define PWM_OUT_MAX        (int16_t)500  // Maximum value is 500
#define PWM_OUT_MIN        (int16_t)(-500) 
#define ROT_VEL_MAX_INC    (real32_t)8.0   // Incresese level: max 3deg/STEER_CONTROL_DT
#define ROT_VEL_MIN_DEC    (real32_t)-8.0  // decrease level

/* PUMP PARAMS */
#define STEER_ANGLE_MIN    (int16_t)-30   // Min set angle (deg)
#define STEER_ANGLE_MAX    (int16_t)30    // Max set angle (deg)
#define PULSE_HOME_POS     (uint16_t)0
//
#define PULSE_FULL360      (int16_t)10000 //(2500*4)=10000 --> 360deg
#define PULSE_TO_ANGLE     (real32_t)0.036  //360/(2500*4)
#define DELTA_PULSE_THRES  (real32_t)10000

#define CTRL_STATE_IDLE    0
#define CTRL_STATE_START   1
#define CTRL_STATE_PID     2
#define CTRL_STATE_STOP    3



/******************************************************************************
 * 							       TYPEDEF                                    *
 ******************************************************************************/


/******************************************************************************
 * 							GLOBAL VARIABLE                                   *
 ******************************************************************************/
extern float Kp, Ki, Kd;
extern int16_t PULSE_PER_REV;
extern BOOL RunEn;

/******************************************************************************
 * 							PRIVATE VARIABLE                                  *
 ******************************************************************************/


/******************************************************************************
 * 							PUBLIC FUNCTION                                   *
 ******************************************************************************/

void steer_init(void);
void steer_set(real32_t *angle);
void steer_get(real32_t *angle);
void car_init(void);

/******************************************************************************
 * 							PRIVATE FUNCTION                                  *
 ******************************************************************************/
void main_control(void);
void car_control(void);
void steer_control(void);


#endif /* __CORE_H */ 

