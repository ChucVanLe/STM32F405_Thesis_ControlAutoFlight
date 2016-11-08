#ifndef __tinhgoc_H
#define __tinhgoc_H
#include "stm32f4xx.h"
// typedef struct
// {
// 	float docao;
// 	float kinhdo;
// 	float vido;
//   struct GOC
// 	{
// 		float x;
// 		float y;
// 		float z;
// 	}goc;
// }PID;
// PID *pid;
// struct PID
// {
// 	struct GOC
// {
// 	float x;
// 	float y;
// 	float z;
// }goc;
// 	float docao;
// 	float kinhdo;
// 	float vido;
// }pid;
double pow (double x,double y); 
float tinhgoc_x(uint8_t *data1,uint8_t *data2);
float tinhgoc_y(uint8_t *data1,uint8_t *data2);
float tinhgoc_z(uint8_t *data1,uint8_t *data2);
#endif

