#include "crc.h"
#include "stdio.h"
#include "stm32f4xx.h"



uint8_t tinhcrc (uint8_t offsetmin,uint8_t *kitu,int offsetmax)
{
uint8_t crc,crclsb;
	int j=0;
	int i1=offsetmin;
	            crc = 0xFF;
          //  byte CRC_LSB;
            for(i1=offsetmin; i1<=offsetmax;i1++)//BO 10 VA 13
            {
                crc ^= *(kitu+i1); //XOR
                for (j=0; j<8; j++)
                {
                    crclsb = (crc & 0x01);
                    crc = ((crc>>1) & 0x7F);
                    if (crclsb == 1)
                    {
                        crc ^= 0xE0;
                    }
                }
            }
            
          return crc & 0xff;
	
}
