/******************************************************************************
 *                                                                            *
 *  @Project    SPRAYING MACHINE                                              *
 *  @Module     \ControlBoard\MDL                                             *
 *	@File       core.c                                                        *
 * 	@Author	    Dang Anh Tung                                                 *
 *  @History                                                                  *
 *              - Mar 27, 14: Initial version                                 *
 *                                                                            *
 ******************************************************************************/
 

/******************************************************************************
 * 							        IMPORT                                    *
 ******************************************************************************/
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "project.h"
/******************************************************************************
 * 							        DEFINE                                    *
 ******************************************************************************/



#define WHEEL_BASE 2.0
#define LOOK_AHEAD_D 3.0//must >= R1
#define GPS_MSG_RATE_S (real32_t)0.1
#define HIGH_SPEED (real32_t)5.0
#define LOW_SPEED (real32_t)2.0
#define PI (real32_t)3.141592654
#define RAD2DEGREE (real64_t)57.295779513
#define DEGREE2RAD (real64_t)0.017453293
#warning check K_STEER
#define K_STEER (real32_t)(300)//=motor angle (degree)/car steer angle (degree)
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
#define OFFSET_LAT	(real64_t)  10.771931
#define OFFSET_LONG (real64_t) 106.658710
#define VEL_TICK	((real64_t)(50000*336)/ENC0_TIM_PRESCALE)
//#define distance2(a,b) (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)
//#define distance(a,b) sqrt(distance2(a,b))
/******************************************************************************
 * 							    GLOBAL VARIABLE                               *
 ******************************************************************************/
real32_t Kp = 0;
real32_t Ki = 0;
real32_t Kd = 0;
int16_t PULSE_PER_REV = 500;
BOOL RunEn = TRUE;
real32_t R1,R2,WPangle;
real64_t angle_factor=0.6;
 volatile real64_t xp, yp, pre_xp, pre_yp;
 volatile real32_t steer_angle=0.0;
 volatile real64_t angle_error=0.0;
volatile real64_t delta_cte;
#define NUM_WAYPOINT (int32_t)6

WAYPOINT_STRUCT WAYPOINT_LLH[NUM_WAYPOINT] = {{ 10.772629,106.659863},
																					{10.772989,106.659670},
																					{ 10.772577,106.658852},
																					{10.772229,106.659032}};
//waypoint hexagon
WAYPOINT_STRUCT WAYPOINT[NUM_WAYPOINT] = {{0, 0, 4.0, 4.0, 60.0},
																					{0, 0, 4.0, 4.0, 60.0},
																					{0, 0, 4.0, 4.0, 60.0},
																					{0, 0, 4.0, 4.0, 60.0},
																					{0, 0, 4.0, 4.0, 60.0},
																					{0, 0, 4.0, 4.0, 60.0}};
/* //waypoint khu cnc 79 wp
WAYPOINT_STRUCT WAYPOINT[NUM_WAYPOINT] = {
{275.3230, 371.4800,1.0, 1.0, 150.0},
{286.1300, 368.2580,1.0, 1.0, 150.0},
{297.8730, 364.1440,1.0, 1.0, 150.0},
{303.9810, 361.9630,1.0, 1.0, 150.0},
{310.0540, 359.3360,1.0, 1.0, 150.0},
{322.1560, 353.8590,1.0, 1.0, 150.0},
{327.9990, 351.1240,1.0, 1.0, 150.0},
{333.7000, 348.3020,1.0, 1.0, 150.0},
{339.4000, 345.1240,1.0, 1.0, 150.0},
{345.1550, 341.5520,1.0, 1.0, 150.0},
{351.0330, 337.9250,1.0, 1.0, 150.0},
{356.9070, 334.1490,1.0, 1.0, 150.0},
{362.4810, 330.0980,1.0, 1.0, 150.0},
{368.0750, 326.1060,1.0, 1.0, 150.0},
{373.6080, 322.0460,1.0, 1.0, 150.0},
{378.9020, 317.8620,1.0, 1.0, 150.0},
{384.1410, 313.6060,1.0, 1.0, 150.0},
{394.0600, 304.6480,1.0, 1.0, 150.0},
{515.4950, 189.3450,1.0, 1.0, 150.0},
{518.7440, 186.4570,1.0, 1.0, 150.0},
{523.6540, 183.5150,1.0, 1.0, 150.0},
{530.0350, 181.6130,1.0, 1.0, 150.0},
{536.0690, 181.5780,1.0, 1.0, 150.0},
{540.2530, 182.6850,1.0, 1.0, 150.0},
{542.8860, 184.0290,1.0, 1.0, 150.0},
{545.6890, 186.3990,1.0, 1.0, 150.0},
{547.9990, 190.1830,1.0, 1.0, 150.0},
{547.9170, 195.4970,1.0, 1.0, 150.0},
{545.7910, 199.8130,1.0, 1.0, 150.0},
{542.5010, 203.7570,1.0, 1.0, 150.0},
{538.8270, 207.5580,1.0, 1.0, 150.0},
{410.4870, 329.2700,1.0, 1.0, 150.0},
{405.9220, 333.3170,1.0, 1.0, 150.0},
{401.1670, 337.3530,1.0, 1.0, 150.0},
{396.2140, 341.2660,1.0, 1.0, 150.0},
{391.0890, 345.1810,1.0, 1.0, 150.0},
{385.9510, 349.1220,1.0, 1.0, 150.0},
{380.8520, 352.9870,1.0, 1.0, 150.0},
{375.5670, 356.6990,1.0, 1.0, 150.0},
{370.1860, 360.2490,1.0, 1.0, 150.0},
{364.7440, 363.8090,1.0, 1.0, 150.0},
{359.1920, 367.3090,1.0, 1.0, 150.0},
{353.5080, 370.5990,1.0, 1.0, 150.0},
{347.6150, 373.7310,1.0, 1.0, 150.0},
{341.4820, 376.8320,1.0, 1.0, 150.0},
{335.4570, 379.9330,1.0, 1.0, 150.0},
{329.4470, 382.6910,1.0, 1.0, 150.0},
{323.1480, 385.3280,1.0, 1.0, 150.0},
{316.6570, 388.0440,1.0, 1.0, 150.0},
{310.1590, 390.5690,1.0, 1.0, 150.0},
{303.6020, 392.8830,1.0, 1.0, 150.0},
{297.0060, 395.0860,1.0, 1.0, 150.0},
{290.2660, 397.1740,1.0, 1.0, 150.0},
{283.3380, 399.2700,1.0, 1.0, 150.0},
{276.4620, 401.1070,1.0, 1.0, 150.0},
{270.9200, 402.5340,1.0, 1.0, 150.0},
{267.6990, 403.4080,1.0, 1.0, 150.0},
{267.0620, 403.5880,1.0, 1.0, 150.0},
{267.0610, 403.5900,1.0, 1.0, 150.0},
{267.0630, 403.5890,1.0, 1.0, 150.0},
{267.0610, 403.5910,1.0, 1.0, 150.0},
{267.0580, 403.5880,1.0, 1.0, 150.0},
{267.0640, 403.5900,1.0, 1.0, 150.0},
{267.0600, 403.5880,1.0, 1.0, 150.0},
{267.0620, 403.5900,1.0, 1.0, 150.0},
{267.0620, 403.5930,1.0, 1.0, 150.0},
{267.0620, 403.5890,1.0, 1.0, 150.0},
{267.0610, 403.5920,1.0, 1.0, 150.0},
{266.9510, 403.6240,1.0, 1.0, 150.0},
{265.2820, 403.9220,1.0, 1.0, 150.0},
{262.3020, 403.9250,1.0, 1.0, 150.0},
{258.2250, 402.6040,1.0, 1.0, 150.0},
{254.6440, 399.0290,1.0, 1.0, 150.0},
{253.2810, 394.1900,1.0, 1.0, 150.0},
{253.5580, 388.6300,1.0, 1.0, 150.0},
{255.6650, 382.8770,1.0, 1.0, 150.0},
{259.7470, 378.0300,1.0, 1.0, 150.0},
{264.9410, 374.6330,1.0, 1.0, 150.0},
{270.2800, 372.9140,1.0, 1.0, 150.0}
};*/																					
/*//mau 1 san a5 luc giac
WAYPOINT_STRUCT WAYPOINT[NUM_WAYPOINT] = {{141, 104, 3.0, 3.0, 122.1},
																					{136.713, 112, 2.5, 2.5, 162.3},
																					{127.486, 120.916, 3.0, 3.0, 139.7},
																					{118.304, 121.5, 4.0, 4.0, 117},
																					{107.538, 103.250, 4.0, 4.0, 109.8},
																					{115.965,93.105, 4.0, 4.0, 119.3},
																					{137, 97, 3.0, 3.0, 130.3}
};																					
*/
																					
/*// mau 2 san banh goc vuong 2												
WAYPOINT_STRUCT WAYPOINT[NUM_WAYPOINT] = {{110.058, 103.310, 3.0, 3.0},
																				{101.636, 117.611, 4.0, 3.0},
																				{80.002, 106.101, 3.0, 3.0},
																				{116.0, 90.0, 3.0, 3.0}};
*/															
/* //san da banh																					
WAYPOINT_STRUCT WAYPOINT[NUM_WAYPOINT] = {{121.057, 75.309, 3.0, 3.0, 90.0},
																				{100.805, 116.869, 4.0, 3.0, 90.0},
																				{15.113, 69.566, 4.0, 3.0, 90.0},
																				{37.5, 30.0, 3.0, 3.0, 90.0}
																				};																					
*/																					
/******************************************************************************
 * 							   PRIVATE VARIABLE                               *
 ******************************************************************************/
real32_t SteerAngleCur = 0; // deg
real32_t SteerAngleSet = 0; // deg
real32_t ctrl_angle = 0, ctrl_angle_1 = 0; //SETPOINT: declare here to debug
static real64_t offset_x, offset_y;
static int32_t wp_idx;
static real64_t wpx1, wpx2, wpy1, wpy2, wpx, wpy, wpxy, inv_dist_wp, slope, m, c;	
static char debug_msg[200];																					
volatile real64_t kp_cte, ki_cte;
extern bool switch_to_control_flight_use_standley;//when receive data of path success--> control_path_use_stanley = true;
extern double utm_lat_long_from_GS[14][2];//contain lat, long in utm coordinates which is uploaded from GS
extern PID_Index Yaw_PID;
/******************************************************************************
 * 							   PUBLIC FUNCTION                                *
 ******************************************************************************/

/******************************************************************************
 * @fn     steer_init     
 * @brief  
 * @retval None
 ******************************************************************************/
void steer_init(void)
{
//    REFTIM_Callback(&main_control);
//    PWM0_Set(0);
//    SteerAngleCur = 0;
//    ENC1_Set(PULSE_HOME_POS);
//    Kp = 20.0;
//    Ki = 0.8;
	
}

/******************************************************************************
 * @fn     steer_set     
 * @brief  
 * @retval None
 ******************************************************************************/
void steer_set(real32_t *angle)
{
    SteerAngleSet = (*angle);//set yaw angle.
}

/******************************************************************************
 * @fn     steer_set     
 * @brief  
 * @retval None
 ******************************************************************************/
void standley_set_yaw_angle(real32_t *angle)
{
    Yaw_PID.SetPoint = (*angle);//set yaw angle.
}
/******************************************************************************
 * @fn     steer_get     
 * @brief  
 * @retval None
 ******************************************************************************/
void steer_get(real32_t *angle)
{
    *angle = SteerAngleCur;
}

real64_t distance(real64_t x1, real64_t y1, real64_t x2, real64_t y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void get_wp_index()
{
	real64_t d_min, d;
	int i;
	for (i = 0; i < NUM_WAYPOINT; i++)//get position flight across from GS
	{
		WAYPOINT[i].x = utm_lat_long_from_GS[i][0];
		WAYPOINT[i].y = utm_lat_long_from_GS[i][1];
	}
	//c2: tinh toa do 6 diem hexagon, bat dau tu vi tri hien tai cua may bay	
//	d_min = distance(xp, yp, WAYPOINT[0].x, WAYPOINT[0].y);
//	wp_idx = 0;
//	
//	for (i = 1; i < NUM_WAYPOINT; i++)
//	{
//		d = distance(xp, yp, WAYPOINT[i].x, WAYPOINT[i].y);
//		if (d < d_min)
//		{
//			d_min = d;
//			wp_idx = i;
//		}
//	}
	//diem thu nhat trong hexagon la vi tri hien tai cua may bay WAYPOINT[0] nen
	//ta khong can tinh d_min, vi tri gan may bay nhat chinh la WAYPOINT[0]
	wp_idx = 0;
	R2 = WAYPOINT[wp_idx].R2;	
	wpx1 = WAYPOINT[wp_idx].x;
	wpy1 = WAYPOINT[wp_idx].y;
	wp_idx = (wp_idx + 1)%NUM_WAYPOINT;//when wp_idx  + 1 = NUM_WAYPOINT, wp_idx = 0
	wpx2 = WAYPOINT[wp_idx].x;
	wpy2 = WAYPOINT[wp_idx].y;
	wpx = wpx2 - wpx1;
	wpy = wpy2 - wpy1;
	wpxy = wpx1 * wpy2 - wpx2 * wpy1;
	inv_dist_wp = 1/sqrt(wpx*wpx + wpy*wpy);
	//slope = atan2(wpy, wpx);//anh Huan: slope hop voi Ox.
	slope = atan2(wpx, wpy);//chuc: slope hop voi Oy
	#warning check wpx==0
	m = wpy / wpx;
	c = wpy1 - m*wpx1; 

	R1 = WAYPOINT[wp_idx].R1;
}
/******************************************************************************
 * @fn     car_init     
 * @brief  
 * @retval None
 ******************************************************************************/
void car_init(void)
{
	char UTMLetter;
	LatLong2UTM(OFFSET_LAT, OFFSET_LONG, &offset_y, &offset_x, &UTMLetter);
	/* offset khu cnc
	offset_x = 696000.0;
	offset_y = 1200000.0;
	*/
/* //uncomment when waypoints are in LLH format
	for (i=0; i<NUM_WAYPOINT; i++)
	{
		LatLong2UTM(WAYPOINT_LLH[i].x, WAYPOINT_LLH[i].y, &WAYPOINT[i].y, &WAYPOINT[i].x, &UTMLetter);
		WAYPOINT[i].x -= offset_x;
		WAYPOINT[i].y -= offset_y;
	}
*/
	wpx1 = WAYPOINT[0].x;
	wpy1 = WAYPOINT[0].y;
	wpx2 = WAYPOINT[1].x;
	wpy2 = WAYPOINT[1].y;
	wp_idx = 1;
	wpx = wpx2 - wpx1;
	wpy = wpy2 - wpy1;
	wpxy = wpx1 * wpy2 - wpx2 * wpy1;
	inv_dist_wp = 1/sqrt(wpx*wpx + wpy*wpy);
	slope = atan2(wpx, wpy);
	#warning check wpx==0
	m = wpy / wpx;
	c = wpy1 - m*wpx1; 
	R1 = WAYPOINT[1].R1;
	R2 = WAYPOINT[0].R2;
#warning delete all lines below	
//	GPSStruct.isavailable = TRUE;
//	IMUStruct.isavailable = TRUE;
//	GPSStruct.quality = GPS_QA_FIX; 
//  GPSStruct.isready = TRUE;
	//car_control();
	//snprintf(debug_msg,200,"x,y,yaw,steer angle,gps speed,hall velocity\r\n");
	//DIS_DMA_Write((uint8_t*)debug_msg, strlen(debug_msg));
}
/******************************************************************************
 * 							PRIVATE FUNCTION                                  *
 ******************************************************************************/

/******************************************************************************
 * @fn     steer_control     
 * @brief  
 * @retval None
 ******************************************************************************/
void steer_control(void)
{
    // STEER CONTROL PARAMTERS
    static uint32_t ctrl_time = 0;  // control period
    static uint32_t ctrl_state = CTRL_STATE_STOP; // START, CTRL, STOP

    static int32_t p_0 = PULSE_HOME_POS, p_1 = PULSE_HOME_POS;
    real32_t dp;
    
    // PID PARAMETERS
    //static real32_t ctrl_angle = 0, ctrl_angle_1 = 0; //SETPOINT
    static real32_t u = 0;
    static real32_t e_0 = 0, e_1 = 0;
    static real32_t Ppart = 0, Ipart = 0, Dpart = 0;
    static real32_t DT = 1/(real32_t)STEER_CONTROL_TIME;
    real32_t rot_vel;

    // TEMP VAR
    int16_t pwm_out = 0;

    ctrl_time++;
    if (ctrl_time < STEER_CONTROL_TIME) return;
    ctrl_time = 0;

    /* ANGLE ESTIMATION */
//    p_0 = ENC1_Get();
    dp = (real32_t)(p_0 - p_1);
    if (dp > DELTA_PULSE_THRES)
        dp -= 65536;
    else if (dp < (-DELTA_PULSE_THRES))
        dp += 65536;
    p_1 = p_0;
    SteerAngleCur += dp * PULSE_TO_ANGLE;

    /* @RunEn: 
     * If enable, set new angle
     * Else stop at once
     */
    if (RunEn == TRUE)
    {
        ctrl_angle = SteerAngleSet;
        // Set point limitation
//        if (ctrl_angle < STEER_ANGLE_MIN) 
//            ctrl_angle = STEER_ANGLE_MIN ;
//        else if (ctrl_angle > STEER_ANGLE_MAX)
//            ctrl_angle = STEER_ANGLE_MAX; 
        ctrl_state = CTRL_STATE_PID;
    }
    else
    {
        ctrl_state = CTRL_STATE_STOP;
    }

    /* @First check the PID parameter (limit,...)
     * Stop motor at once
     */
    if ((Kp == 0) && (Ki == 0) && (Kd == 0))
    {
        ctrl_state = CTRL_STATE_STOP;
    }

    /* @PID state: 
     */
    if (ctrl_state == CTRL_STATE_PID)
    {
        // Rotation angle velocity saturation control
        rot_vel = ctrl_angle - ctrl_angle_1;
        if (rot_vel > ROT_VEL_MAX_INC) 
            ctrl_angle_1 += ROT_VEL_MAX_INC;
        else if (rot_vel < ROT_VEL_MIN_DEC) 
            ctrl_angle_1 += ROT_VEL_MIN_DEC;
        else ctrl_angle_1 = ctrl_angle;
        
        //ctrl_angle_1 = ctrl_angle;

        // PID control
        e_0 = (ctrl_angle_1 - SteerAngleCur);
        Ppart = Kp * e_0;
        Ipart += Ki * DT * (e_0 + e_1) / 2; // integration approximation
        Dpart = Kd * (e_0 - e_1) / DT;
        u = Ppart + Ipart + Dpart;
        e_1 = e_0;

        // PWM saturation
        if (u > PWM_OUT_MAX) u = PWM_OUT_MAX;
        else if (u < PWM_OUT_MIN) u = PWM_OUT_MIN;
    }
    
    /* @STOP state: 
     * Stop motor at once, reset all variables
     */
    if (ctrl_state == CTRL_STATE_STOP)
    {
        // Stop period
        ctrl_angle = 0;
        ctrl_angle_1 = 0;
        Ppart = 0;
        Ipart = 0;
        Dpart = 0;
        e_1 = 0;
        u = 0;
    }
    
    // PWM output
    pwm_out = (int16_t)u;
//    PWM0_Set(pwm_out);
}
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const real64_t *pos, real64_t *r)
{
    real64_t sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
    real64_t e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
    
    r[0]=(v+pos[2])*cosp*cosl;
    r[1]=(v+pos[2])*cosp*sinl;
    r[2]=(v*(1.0-e2)+pos[2])*sinp;
}
real32_t persuit_control(real64_t xp, real64_t yp, real64_t dir)
{
	real64_t A, B, C, x1, x2, xg, yg;
	A = 1 + m*m;
	B = -2*xp + 2*m*c - 2*m*yp;
	C = xp*xp + (c-yp)*(c-yp) - LOOK_AHEAD_D * LOOK_AHEAD_D;
	#warning check delta<0
	x1 = (-B + sqrtf(B*B-4*A*C)) / (2*A);
	x2 = (-B/A) - x1;
	#warning check wpx = 0
	if ((x2-x1) * wpx>=0)
	{
		xg = x2;
		yg = m*x2 + c;
	}
	else
	{
		xg = x1;
		yg = m*x1 + c;
	}
	A = (xg-xp)*sinf(dir) - (yg-yp)*cosf(dir);
	return atanf(-2*WHEEL_BASE*A/(LOOK_AHEAD_D*LOOK_AHEAD_D));
}
real64_t SubMod(real64_t angle1, real64_t angle2)
{
	volatile real64_t tmp;
	tmp = angle1 - angle2;
	if (tmp < -1.0*PI)
		tmp = tmp + 2.0*PI;
	else if (tmp > 1.0*PI)
		tmp = tmp - 2.0*PI;
	return tmp;
}
real32_t stanley_control(real64_t cte, real64_t cteI, real64_t dir)
{					
	volatile real64_t delta_cte;


/*
	if (ki_cte * cteI > 0.5)
		cteI = 0.5/ki_cte;
	else if (ki_cte * cteI < -0.5)
		cteI = -0.5/ki_cte;
*/
	
	delta_cte = atan(kp_cte * cte + ki_cte * cteI);
	return (0.6*angle_error + delta_cte) * RAD2DEGREE;
}

/******************************************************************************
 * @fn     car_control     
 * @brief  Position controller
 * @retval None
 ******************************************************************************/
void car_control(void)
{

		// Wait until get valid data from 2 sensors GPS & IMU
    if (!(GPSStruct.isavailable && IMUStruct.isavailable)) return; 
    if ((GPSStruct.quality != GPS_QA_NOT_FIX) &&
       (GPSStruct.isready))
    {
				static volatile real64_t cteI = 0.0;
				volatile real64_t fdist, cte;
				char UTMLetter;
				volatile real64_t angle_error, dir;
				volatile real32_t heading;
				volatile real64_t velocity;
				static volatile bool first_pos = true;
				
//				velocity = VEL_TICK/(real64_t)ENC0_Get();//km/h
				// GPS signal is ok
        GPSStruct.isready = FALSE; // clear to wait until next string

			// Controller here
				#warning get xp,yp from gps msg

				LatLong2UTM(GPSStruct.latitude, GPSStruct.longitude, 
										&yp, &xp, &UTMLetter);
				//test
//					LatLong2UTM(10.876543, 106.6789, 
//										&yp, &xp, &UTMLetter);
				//result --ok			
				xp -= offset_x;
				yp -= offset_y;
//when receive hexagon from GS-->control_path_use_stanley = true.
				if (switch_to_control_flight_use_standley)
				{
					switch_to_control_flight_use_standley = false;
					//get hexagon from GS
					get_wp_index();//waypoint is point flight accross
				}
			
        fdist = sqrt((xp - wpx2) * (xp - wpx2) + (yp - wpy2) * (yp - wpy2));
				if (fdist < R1)//change trajectory								
        {
					kp_cte = 0.05;
					//kp_cte = 0.2;
					ki_cte = 0.0;					
					
					//kp_cte = 0.4 - (180 - WAYPOINT[wp_idx].WPangle - 5.0)*0.4/90 + 0.05;
					//ki_cte = 0.02 - (180 - WAYPOINT[wp_idx].WPangle - 5.0)*0.02/90 + 0.00002;
					R2 = WAYPOINT[wp_idx].R2;
//					LED_Set(LED_RED, ON);
					wp_idx++;
					if (wp_idx >= NUM_WAYPOINT)
						wp_idx = 0;
					wpx1 = wpx2;
					wpy1 = wpy2;
					wpx2 = WAYPOINT[wp_idx].x;
					wpy2 = WAYPOINT[wp_idx].y;
					wpx = wpx2 - wpx1;
					wpy = wpy2 - wpy1;
					wpxy = wpx1 * wpy2 - wpx2 * wpy1;
					inv_dist_wp = 1/sqrt(wpx*wpx + wpy*wpy);
					slope = atan2(wpx, wpy);
					m = wpy / wpx;
					c = wpy1 - m*wpx1;
					
					R1 = WAYPOINT[wp_idx].R1;
					
				}
				else
				{
//					LED_Set(LED_RED, OFF);
				}
				
				cte = (wpy*xp - wpx*yp - wpxy) * inv_dist_wp;//khong cach tu vi tri hien tai may bay toi duong quy dao
				if (fabs(GPSStruct.speed) > 0.4)//is moving?
				{
					cteI += GPS_MSG_RATE_S*cte;
				}
				
				//dir = IMUStruct.yaw * DEGREE2RAD;
				//do may bay troi nen goc yaw khac heading, ta lay goc heading la huong cua vecto van toc
				dir = GPSStruct.heading * DEGREE2RAD;
				while (fabsf((float)dir) > PI)//-PI <= yaw <= PI
				{
					if (dir < -1.0*PI)
						dir += 2*PI;
					else if (dir > PI)
						dir -= 2*PI;
				}
				angle_error = slope - dir;//??  --> ok

				if (angle_error < -1.0*PI)
					angle_error = angle_error +2*PI;
				else if (angle_error > PI)
					angle_error = angle_error -2*PI;
				
				fdist = sqrt((xp - wpx1)*(xp - wpx1) + (yp - wpy1)*(yp - wpy1));
				
				if ((fdist < R2) && (fabs(angle_error)>PI/9))//neu vi tri may bay trong vung R2 thi tat Ki
				{	
					#warning neu goc lon thi k nen tat
					cteI = 0.0;
//					LED_Set(LED_BLUE, ON);
					//steer_angle = persuit_control(xp, yp, dir);
					//if (fabs(GPSStruct.speed) > 0.3)//is moving?
					//{
	
					delta_cte = atan(kp_cte * cte + ki_cte * cteI);
					steer_angle = (0.4*angle_error + delta_cte) * RAD2DEGREE;
					//	if (steer_angle > 30)
					//		steer_angle = 30;
					//	else if (steer_angle < -30)
					//		steer_angle = -30;
					//}
				}
				else
				{
					kp_cte = 0.25;
					ki_cte=0.04;
					//ki_cte = 0.005;
//					LED_Set(LED_BLUE, OFF);
					//steer_angle = persuit_control(xp, yp, dir);
					//if (fabs(GPSStruct.speed) > 0.3)//is moving?
					//{
					delta_cte = atan(kp_cte * cte + ki_cte * cteI);
					steer_angle = (0.4*angle_error + delta_cte) * RAD2DEGREE;
					//	if (steer_angle > 30)
					//		steer_angle = 30;
					//	else if (steer_angle < -30)
					//		steer_angle = -30;
					//}
				}

//				snprintf(debug_msg,200,"%11.3f,%11.3f,%6.2f,%11.3f,%11.3f\r\n"
//					,xp,yp,IMUStruct.yaw,wpx1,wpx2);
			
				steer_angle *= K_STEER;	
        // Set steer angle
        //if (fabs(GPSStruct.speed) > 0.3)//is moving?
				//{ 
					steer_set(&steer_angle);
				//}

    }			
}


/******************************************************************************
 * @fn     main_control     
 * @brief  This function will be called every 1ms
 *         All controllers should be here
 * @retval None
 ******************************************************************************/
void main_control(void)
{
    car_control();
    //steer_control();
}


