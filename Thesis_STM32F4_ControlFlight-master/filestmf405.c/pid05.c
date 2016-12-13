#include "project.h"
#include "math.h"
PID_Index Roll_PID;
PID_Index Pitch_PID;
PID_Index Yaw_PID;
PID_Index Alt_PID;
PID_Index Latitude;
PID_Index Longitude ;
PID_Index Speed;

PID_Index Press;

float init_width_pulse_CH3;
void PID_Init(void)
{
	Roll_PID.Error = Roll_PID.PreError = 0;
	Roll_PID.Kp = 0.0001;
	Roll_PID.Ki = 0.0000015;
	Roll_PID.Kd = 0.000001;
	Roll_PID.Switch_manual_auto = false;
	Pitch_PID.Error = Pitch_PID.PreError = 0;
	Pitch_PID.Kp = 0.0001;
	Pitch_PID.Ki = 0.0000015;
	Pitch_PID.Kd = 0.000001;
	Pitch_PID.Switch_manual_auto = false;
	Yaw_PID.Error = Yaw_PID.PreError = 0;
	Yaw_PID.Kp = 0.00005;
	Yaw_PID.Ki = 0.0000015;
	Yaw_PID.Kd = 0.000001;
	Yaw_PID.Switch_manual_auto = false;
	Alt_PID.Error = Alt_PID.PreError = 0;
	Alt_PID.Kp = 0.00005;
	Alt_PID.Ki = 0.0000015;
	Alt_PID.Kd = 0.000001;
	Alt_PID.SetPoint = 40;
	Alt_PID.Switch_manual_auto = false;
}
float Time_sample = 0.01;
//*****************************************************
//get speed
void Sampling_VTG(char* VTG , int lenght)
{
	int i = 0;
	char temp_speed[7];
	int comma = 0;//find ','
	int fp_speed =0, ep_speed = 0;
	for (i=0;i<lenght;i++)
	{
			if (VTG[i] == ',')
			{
				comma++;
				if(comma == 7)
					fp_speed = i + 1;
				if(comma == 8)
				{
					ep_speed = i;
					break;
				}
			}
	 }
	if(ep_speed!=(fp_speed))
	{
	strncpy(temp_speed, &VTG[fp_speed], ep_speed - fp_speed);
	Speed.Current = atof(temp_speed);
	}
}
//*****************************************************
//get latitude, longitude, altitude
void Sampling_GGA(char* GGA , int lenght)
{
	char min_lat[10], degree_lat[3], min_lon[10], degree_lon[3], temp_alt[5];
	uint8_t pos_dot_lat, pos_dot_lon;
	int i = 0;
	int comma = 0;//find ','
	int fp_alt = 0, fp_latitude=0, fp_longitude=0;
	int ep_alt = 0, ep_latitude=0, ep_longitude=0;

	for (i=0;i<lenght;i++)
	{
			if (*(GGA+i)==',')
			{
				comma++;
				if(comma==2)
					fp_latitude = i + 1;
				if(comma==3)
					ep_latitude = i;
				if(comma==4)
					fp_longitude = i + 1;
				if(comma==5)
					ep_longitude = i;
				if (comma==9)
					fp_alt = i + 1;
				if (comma==10)
				{
					ep_alt = i;
					break;
				}
			}
	}
	//find position dot latitude, longitude
	for (i = fp_latitude; i < ep_latitude; i++)
	{
			if (*(GGA+i)=='.') pos_dot_lat = i;
	}
	for (i = fp_longitude; i < ep_longitude; i++)
	{
			if (*(GGA+i)=='.') pos_dot_lon = i;
	}
	
	if(ep_latitude!=(fp_latitude))
	{
		strncpy(degree_lat, &GGA[fp_latitude], pos_dot_lat - 2 - fp_latitude);
		strncpy(min_lat, &GGA[pos_dot_lat - 2], ep_latitude - (pos_dot_lat - 2));
		Latitude.Current = atof(degree_lat) + atof(min_lat) / 60;
	}
	else Latitude.Current = 0;

	if(ep_longitude!=(fp_longitude))
	{
		strncpy(degree_lon, &GGA[fp_longitude], pos_dot_lon - 2 - fp_longitude);
		strncpy(min_lon, &GGA[pos_dot_lon - 2], ep_longitude - (pos_dot_lon - 2));
		Longitude.Current = atof(degree_lon) + atof(min_lon) / 60;
	}
	else Longitude.Current = 0;

	// neu co thong tin do cao thi lam 
	//if(state_alt==1)
	{
		if (ep_alt != (fp_alt))
		{
			strncpy(temp_alt, &GGA[fp_alt], ep_alt - fp_alt);
			Alt_PID.Current = atof(temp_alt);
			Alt_PID.Enable = 1 ;
		}
		else Alt_PID.Current = 0;
	 }
}
//*****************************************************
//get roll, pitch, yaw
void Sampling_RPY(uint8_t* IMU , int lenght)
{
	int i = 0 ; 
	float roll=0,pitch=0,yaw=0,press=0;
	// cap nhat roll
	for (i=4;i<8;i++)
		roll+= (*(IMU+i)-0x30)*pow(10,(7-i));
	roll=roll*0.1;
	if (*(IMU+3)=='-')
		roll=-roll;
	Roll_PID.Current=roll;
	Roll_PID.Enable=1;
	//cap nhat pitch
	for (i=10;i<14;++i)
	pitch+= (*(IMU+i)-0x30)*pow(10,(13-i));
	pitch=pitch*0.1;
	if (*(IMU+9)=='-')
		pitch=-pitch;
	Pitch_PID.Current=pitch;
	Pitch_PID.Enable =1 ;
	//cap nhat Yaw
	for (i=16;i<20;++i)
	yaw+= (*(IMU+i)-0x30)*pow(10,(19-i));
	yaw=yaw*0.1;
	if (*(IMU+15)=='-')
		yaw=-yaw;
	Yaw_PID.Current=yaw;
	Yaw_PID.Enable =1 ;	
// 	if(state_press==1)
// 	{	
	for (i=76;i<80;++i)
	press+= (*(IMU+i)-0x30)*pow(10,(79-i));
	press+=100000;
	press=1- pow((press/101325),(1/5.25578));
	press=(press*pow(10,5))/2.25577;
	Press.Current=press;
	//chuyen doi press sang do cao
//	if(state_press==1)
//	{
//	Alt_PID.Current = Press.Current;
//	Alt_PID.Enable =1 ;
//	}
}

/*******************************************************************************
Function name: Call_Roll_PID
Decription: 
Input: v_set
Output: None

//Formula
a0=Kp+(Ki*T)/2+Kd/T;
a1=-Kp+(Ki*T)/2-(2*Kd)/T;
a2=Kd/T;
uk = u(k - 1)+ a0 * e(k) + a1 * e(k - 1) + a2 * e(k - 2)
*******************************************************************************/
void Call_Roll_PID(float Setpoint)
{
	
//		if(Roll_PID.Enable)
//		TIM4->CCR1 = Call_PID_Controller_Angle(&Roll_PID.Enable, Setpoint, Roll_PID.Current, &Roll_PID.PartKi,
//		Roll_PID.Kp, Roll_PID.Ki, Roll_PID.Kd, 0.01, &Roll_PID.Pid_Result, &Roll_PID.PreError,
//		1.5, 0.4, -0.42);
		if(Roll_PID.Enable)
	{
		//at manual --> auto, get setpoint is current

		Roll_PID.Enable = 0;
		Roll_PID.Error = Setpoint - Roll_PID.Current;

		//limit -180, 180
		if (Roll_PID.Error > 180)
			Roll_PID.Error -= 360;
		if (Roll_PID.Error < -180)
			Roll_PID.Error += 360;
		//PID_ThanhTan
		//Roll_PID.Pid_Result_Temp=Roll_PID.Pid_Result;
		//Roll_PID.Pid_Result=Roll_PID.Pid_Result_Temp+Roll_PID.a0*Roll_PID.e[2]+Roll_PID.a1*Roll_PID.e[1]	+Roll_PID.a2*Roll_PID.e[0];
		//new pid controller
		Roll_PID.PartKi += Roll_PID.Ki * Roll_PID.Error * Time_sample;
		//PartKp = Kp * Error
		//PartKi += Ki * Error * Time_sample;
		//PartKd = Kd * (Error - PreError) / Time_sample;
		Roll_PID.Pid_Result += Roll_PID.Kp * Roll_PID.Error + Roll_PID.PartKi + Roll_PID.Kd * (Roll_PID.Error - Roll_PID.PreError) / Time_sample;
		//limit
		if(Roll_PID.Pid_Result > 0.4) Roll_PID.Pid_Result = 0.4;
		if(Roll_PID.Pid_Result < -0.42) Roll_PID.Pid_Result = -0.42;
		// Dat gia tri vao PWM	
		Gent_Pwm_Roll(-Roll_PID.Pid_Result);
				
		Roll_PID.PreError = Roll_PID.Error;
	}
	else return;
}

/*******************************************************************************
Function name: Call_Pitch_PID
Decription: 
Input: 
Output: None
*******************************************************************************/
void Call_Pitch_PID(float Setpoint)
{
		if(Pitch_PID.Enable)
		TIM4->CCR2 = Call_PID_Controller_Angle(&Pitch_PID.Enable, Setpoint, Pitch_PID.Current, &Pitch_PID.PartKi,
		Pitch_PID.Kp, Pitch_PID.Ki, Pitch_PID.Kd, 0.01, &Pitch_PID.Pid_Result, &Pitch_PID.PreError,
		1.56, 0.48, -0.34);
}

/*******************************************************************************
Function name: Call_Yaw_PID
Decription: Ham PID vi tri dong co theo gia tri xung encoder
Input: p_set
Output: None
*******************************************************************************/
void Call_Yaw_PID(float Setpoint)
{
		if(Yaw_PID.Enable)
		TIM4->CCR4 = Call_PID_Controller_Angle(&Yaw_PID.Enable, Setpoint, Yaw_PID.Current, &Yaw_PID.PartKi,
		Yaw_PID.Kp, Yaw_PID.Ki, Yaw_PID.Kd, 0.01, &Yaw_PID.Pid_Result, &Yaw_PID.PreError,
		1.52, 0.44, -0.4);
}
/*******************************************************************************
Function name: Call_Alt_PID
Decription: Ha
Input: p_set
Output: None
*******************************************************************************/
void Call_Alt_PID(float Setpoint)
{
//		if(Alt_PID.Enable)
//		TIM4->CCR3 = Call_PID_Controller_Angle(&Alt_PID.Enable, Setpoint, Alt_PID.Current, &Alt_PID.PartKi,
//		Alt_PID.Kp, Alt_PID.Ki, Alt_PID.Kd, 0.1, &Alt_PID.Pid_Result, &Alt_PID.PreError,
//		init_width_pulse_CH3 / 1000, 1.92 - init_width_pulse_CH3 / 1000, 1.110 - init_width_pulse_CH3 / 1000);
	//simulate
			if(Alt_PID.Enable)
		TIM4->CCR3 = Call_PID_Controller_Angle(&Alt_PID.Enable, Setpoint, Alt_PID.Current, &Alt_PID.PartKi,
		Alt_PID.Kp, Alt_PID.Ki, Alt_PID.Kd, 0.1, &Alt_PID.Pid_Result, &Alt_PID.PreError,
		init_width_pulse_CH3 / 1000, 1.26 - init_width_pulse_CH3 / 1000, 1.110 - init_width_pulse_CH3 / 1000);
}
/************************************************************************************************/
//TIM4->CCR3 = Pwm;//channel 3
//TIM4->CCR2 = Pwm;//channel 2
void Gent_Pwm_Roll(float Roll)
{
	int Pwm ;
 	//Pwm =(int)((1+(Roll+45)/90)*21711/8);
//	Pwm =(int)((float)(1.5 - Roll/90) * 21711 / 13.7);
	Pwm =(int)((float)(1.5 + Roll) * 21711 * 73 / 1000);
	//1.08-1.5 <=  Roll <= 1.85 - 1.5
	if(Pwm > 2932) Pwm = 2932;//Pulse = 1.85ms
	if(Pwm < 1712) Pwm = 1712;//Pulse = 1.08ms
	TIM4->CCR1 = Pwm;
//	TIM4->CCR1 = 1584;//1ms
//	TIM4->CCR1 = 2377;//1.5ms,dung yen
	//Tpwm = 21177 pulse
	//f = 73Hz
}
void Gent_Pwm_Pitch(float Pitch)
{
	int Pwm ;
 	Pwm =(int)((float)(1.56 + Pitch) * 1584903 /1000);
	//1.15 - 1.56 <= Pitch <= 1.85 - 1.56
	if(Pwm > 2932) Pwm = 2932;//Pulse = 1.85ms
	if(Pwm < 1822) Pwm = 1822;//Pulse = 1.15ms
	TIM4->CCR2 = Pwm;
}
void Gent_Pwm_Yaw(float Yaw)
{
	int Pwm ;
	Pwm =(int)((float)(1.5 + Yaw) * 21711 * 73 / 1000);
	//1.24 - 1.5 <= Yaw <= 1.9 - 1.5
	//limit
	if(Pwm > 3011) Pwm = 3011;//Pulse = 1.9ms
	if(Pwm < 1965) Pwm = 1965;//Pulse = 1.24ms
	TIM4->CCR4 = Pwm;
}
void Gent_Pwm_Alt(float Alt)
{
	int Pwm ;
	Pwm =(int)((float)(1.5 + Alt) * 21711 * 73 / 1000);
	//limit
	//1.110 - 1.5 <= Alt <= 1.92 - 1.5
//	if(Pwm > 3043) Pwm = 3043;//Pulse = 1.92ms
//	if(Pwm < 1759) Pwm = 1759;//Pulse = 1.110ms
	//test simulate
	if(Pwm > 2694) Pwm = 2694;//Pulse = 1.7ms
	if(Pwm < 1759) Pwm = 1759;//Pulse = 1.110ms
	TIM4->CCR3 = Pwm;
}

int32_t Call_PID_Controller_Angle(bool *Enable, float Setpoint, float Current, float *PartKi, 
	float Kp, float Ki, float Kd, float	Time_sample_GPS, float *Pid_Result, float *PreError, 
		float init_center_value, float limit_up_output, float limit_down_output)
{
		float Error; 
		if(*Enable)
		{
				//at manual --> auto, get setpoint is current
			*Enable = 0;
			Error = Setpoint - Current;

				//limit -180, 180
			if (Error > +180) Error -= 360;
			if (Error < -180) Error += 360;

				//at manual --> auto, get setpoint is current

				//new pid controller
				*PartKi += Ki * Error * Time_sample_GPS;
				//PartKp = Kp * Error
				//PartKi += Ki * Error * Time_sample;
				//PartKd = Kd * (Error - PreError) / Time_sample;
				*Pid_Result += Kp * Error + *PartKi + Kd * (Error - *PreError) / Time_sample_GPS;
				//limit at real time
				if(*Pid_Result > limit_up_output) *Pid_Result = limit_up_output;
				if(*Pid_Result < limit_down_output) *Pid_Result = limit_down_output;

				// Dat gia tri vao PWM	
				return (int)((float)(init_center_value + *Pid_Result) * 21711 * 73 / 1000);
				*PreError = Error;
		}
}
