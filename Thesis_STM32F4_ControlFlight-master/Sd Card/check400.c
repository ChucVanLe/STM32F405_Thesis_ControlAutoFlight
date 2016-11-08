 #include "stm32f4xx.h"
 #include "diskio.h"
 #include "ff.h"
 #include "integer.h"
 #include "string.h"
 #include <stdio.h>
 #include <stdlib.h>
  BYTE buff[64]; 
  uint8_t Message1[] ={0x35,0x37};


 void Delay(__IO uint32_t nCount)
 {
   while(nCount--)
   {
   }
 }


 int main(void)
 {  
   
   int i,j;
static		FRESULT st_Result;   /* Result code */  
static		FATFS fatfs;  /* File system object */  
static		FIL fil;   /* File object */  
static		DIR dir;   /* Directory object */  
static		FILINFO fno;  /* File information object */  
static		UINT *bw;  
   UCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
   
   
   power();
 	
   f_mount(0, &fatfs);  
   f_mkdir ("0:/HTDKN2");	
 	
   st_Result = f_open(&fil, FilePath, FA_WRITE | FA_OPEN_ALWAYS);
   for(i=0;i<400;++i)
   st_Result = f_write(&fil, Message1, 1, bw);

 	Delay(84000000);

 	for(i=0;i<1400;++i)
   st_Result = f_write(&fil, Message1, 1, bw);
   
   st_Result = f_close(&fil); 

 }



