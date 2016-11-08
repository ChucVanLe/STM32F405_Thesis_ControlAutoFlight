// #include "tinhgps.h"
// extern int i,dem_phay;
// int i2,dau_cham;//i2 la bien dung de dem kinh vi do
// int dau_kinhdo;
// float kinhdo,vido,docao,vantoc; //T sua
// int dau_vido;
// int dau_docao=0,cuoi_docao=0;
// int dau_vantoc=0,cuoi_vantoc=0; 
// uint8_t vi_do[9],kinh_do[10],do_cao[10],van_toc[10];
// float gps_vido(uint8_t *data)
// {

//  if(dem_phay==2)
//  {
//   vido=0;
//   dau_vido=i;
//   if (*(&data[dau_vido+1])==',')
//   {
//    vido=0;
//    //return vido;
//   }
//   else
//   {
// 	for(i2=1;i2<10;i2++)
// 	{
// 	 vi_do[i2-1]=*(&data[dau_vido +i2]);
// 	}
// 	for(i2=0;i2<4;i2++)
// 	 {
// 	  vido +=(vi_do[i2]-0x30)* pow(10,(7-i2));
// 	 }
// 	 for(i2=5;i2<9;i2++)
// 	 {
// 	  vido +=(vi_do[i2]-0x30)* pow(10,(8-i2));
// 	 }
// 	 vido /=10000;
// 	 //return vido;
// 	}
//   }
// 	return vido;
//  }
//  
//  
//  

// float gps_kinhdo(uint8_t *data)
// {
//  if(dem_phay==4)
//  {
//   kinhdo=0;
// 	dau_kinhdo=i;
// 	if(*(&data[dau_kinhdo+1])==',')
// 	{
// 	 kinhdo=0;
// 	 //return kinhdo;
// 	}
// 	else
// 	{
// 		for(i2=1;i2<11;i2++)
// 		{
// 		 kinh_do[i2-1]= *(&data[dau_kinhdo+i2]);
// 		}
// 		
// 		for(i2=0;i2<5;i2++)
// 		{
// 		 kinhdo += (kinh_do[i2]-0x30) * pow(10,(8-i2));
// 		}
// 		for(i2=6;i2<10;i2++)
// 		{
// 		 kinhdo += (kinh_do[i2] -0x30) *pow(10,(9-i2));
// 		}
// 		kinhdo /=10000;
// 		//return kinhdo;
// 	}
//  }
//     return kinhdo;

// }


// float gps_docao(uint8_t *data)
// {
//   if(dem_phay==9)
//   {
// 	dau_docao=i;
//   }
//   if(dem_phay==10)
//   {
// 		docao=0;
//     cuoi_docao=i;
// 		if(*(&data[dau_docao+1])== *(&data[cuoi_docao]))
// 		{
// 		docao=0;
// 		//return docao;
// 		}
// 		else
// 		{
// 		 for(i2=dau_docao +1;i2<cuoi_docao;i2++)
// 		 {
// 			do_cao[i2-dau_docao-1]=*(&data[i2]);
// 		 }
// 		 for(i2=0;i2<cuoi_docao-dau_docao;i2++)
// 		 {
// 			if(do_cao[i2]=='.')
// 			{
// 			dau_cham=i2;
// 			for(i2=0;i2<dau_cham;i2++)
// 			{
// 			 docao +=(float)(do_cao[i2]-0x30)* pow(10,(dau_cham - i2-1));
// 			}
// 			for(i2=dau_cham+1;i2<cuoi_docao-dau_docao;i2++)
// 			{
// 			 docao += (float)((do_cao[i2]-0x30) * pow(10,dau_cham-i2));
// 			}
// 	   }
// 	  }
// 		//return docao;
// 	 }
// 	 }
// 	 return docao;
// 	
// }

// float gps_vantoc (uint8_t *data)
// {
// 	
// }







