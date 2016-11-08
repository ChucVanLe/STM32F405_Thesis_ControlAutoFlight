#include "tinhgoc.h"

	float tinhgoc_x(uint8_t *data1,uint8_t *data2)	
	{
		int i2;
		float gocx=0;
		for(i2=0;i2<5;i2++)
		{
			*(&data1[i2])=*(&data2[i2+1]);
		}
		 gocx=0;
		for(i2=1;i2<5;i2++)
		{
			gocx += (*(&data1[i2])-0x30)* pow(10,(4-i2));
		}
		 gocx /=10;
		if(*(&data1[0])=='-')
		{
			 gocx *= -1;
		}
		return  gocx;
	}
	float tinhgoc_y(uint8_t *data1,uint8_t *data2)	
	{
		int i;
		 float gocy=0;
		//lay goc theo truc y
		for(i=0;i<5;i++)
		{
			*(&data1[i])=*(&data2[i+7]);
		}
		gocy=0;
		for(i=1;i<5;i++)
		{
			gocy += (*(&data1[i])-0x30)* pow(10,(4-i));
		}
		gocy /=10;
		if(*(&data1[0])=='-')
		{
		gocy *= -1;
		}
		return gocy;
	}
	float tinhgoc_z(uint8_t *data1,uint8_t *data2)	
	{
		int i;
		float gocz=0;
		//lay goc theo truc z
		for(i=0;i<5;i++)
		{
			*(&data1[i])=*(&data2[i+13]);
		}
		gocz=0;
		for(i=1;i<5;i++)
		{
			gocz +=(*(&data1[i])-0x30)* pow(10,(4-i));
		}
		gocz /=10;
		if(*(&data1[0])=='-')
		{
			gocz *= -1;
		}
		return gocz;
	}
					//lay goc theo truc x.
// 				for(i=0;i<5;i++)
// 				{
// 					x_IMU[i]=Buffer1[i+1];
// 				}
// 				pid.goc.x= 0;
// 				for(i=1;i<5;i++)
// 				{
// 					pid.goc.x +=(x_IMU[i]-0x30)* pow(10,(4-i));
// 				}
// 				pid.goc.x /=10;
// 				if(x_IMU[0]=='-')
// 				{
// 					pid.goc.x *= -1;
// 				}
// 				//lay goc theo truc y
// 				for(i=0;i<5;i++)
// 				{
// 					y_IMU[i]=Buffer1[i+7];
// 				}
// 				pid.goc.y=0;
// 				for(i=1;i<5;i++)
// 				{
// 					pid.goc.y += (y_IMU[i]-0x30)* pow(10,(4-i));
// 				}
// 				pid.goc.y /=10;
// 				if(y_IMU[0]=='-')
// 				{
// 					pid.goc.y *= -1;
// 				}
				
// 				//lay goc theo truc z
// 				for(i=0;i<5;i++)
// 				{
// 					z_IMU[i]=Buffer1[i+13];
// 				}
// 				pid.goc.z=0;
// 				for(i=1;i<5;i++)
// 				{
// 					pid.goc.z +=(z_IMU[i]-0x30)* pow(10,(4-i));
// 				}
// 				pid.goc.z /=10;
// 				if(z_IMU[0]=='-')
// 				{
// 					pid.goc.z *= -1;
// 				}

// void GPIO_Configuration(void);

// 	float chartofloat(uint8_t *abc)
// 	{
// 		float tmp;
// 		tmp = (float)((abc[1]-0x30)*(10^3) + (abc[2]-0x30)*10^2 + (abc[3]-0x30)*10);
// 		if(abc[0]=='-') tmp*=-1;
// 		return tmp/10;
// 	}






