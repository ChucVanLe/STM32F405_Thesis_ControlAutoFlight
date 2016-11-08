#ifndef __tinhgpsnew_H
#define __tinhgpsnew_H
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
float gps_vido(uint8_t *data1,uint8_t *data2,int lenght);
float gps_kinhdo(uint8_t *data1,uint8_t *data2,int lenght);
float gps_docao(uint8_t *data1,uint8_t *data2,int lenght);
float gps_vantocthang(uint8_t *data1,uint8_t *data2,int lenght);
#endif






