#include "tinhgpsnew.h" 
float gps_vido(uint8_t *data1,uint8_t *data2,int lenght)
{
	int i=0,i2=0,demphay=0,dauvido=0;
	float vido=0;
	for(i=0;i<lenght;i++)
	{
		if(*(&data2[i])==',')
		{
			demphay++;
			if(demphay==2)
			{
				dauvido=i;
				if(*(&data2[dauvido+1])==',')
				{
					vido=0;
				}
				else
				{
					for(i2=0;i2<4;i2++)
					{
						*(&data1[i2])= *(&data2[dauvido+1+i2]);
						vido += (*(&data1[i2])-0x30) * pow(10,7-i2);
					}
					for(i2=5;i2<9;i2++)
					{
						*(&data1[i2])= *(&data2[dauvido+1+i2]);
						vido += (*(&data1[i2])-0x30) * pow(10,8-i2);
					}
					vido /=10000;
				}
			}
		}
	 }
	 return vido;
	}
	
float gps_kinhdo (uint8_t *data1,uint8_t *data2,int lenght)
{
	int i=0,i2=0,demphay=0,daukinhdo=0;
	float kinhdo=0;
	for(i=0;i<lenght;i++)
	{
		if(*(&data2[i])==',')
		{
			demphay++;
			if(demphay==4)
			{
				daukinhdo=i;
				if(*(&data2[daukinhdo+1])==',')
				{
					kinhdo=0;
				}
				else
				{
					for(i2=0;i2<5;i2++)
					{
						*(&data1[i2])= *(&data2[daukinhdo+1+i2]);
						kinhdo += (*(&data1[i2])-0x30) * pow(10,8-i2);
					}
					for(i2=6;i2<10;i2++)
					{
						*(&data1[i2])= *(&data2[daukinhdo+1+i2]);
						kinhdo += (*(&data1[i2])-0x30) * pow(10,9-i2);
					}
					kinhdo /=10000;
				}
			}
		}
	}
	return kinhdo;
}

float gps_docao (uint8_t *data1,uint8_t *data2,int lenght)
{
	int i=0,i2=0,demphay=0,daudocao=0,cuoidocao=0,daucham=0,count=0;
	float docao=0;
	for(i=0;i<lenght;i++)
	{
		if(*(&data2[i])==',')
		{
			demphay++;
			if(demphay==9)
			{
				daudocao=i;
				if(*(&data2[daudocao+1])==',')
				{
					docao=0;
				}
				else
				{
					for(i2=0;i2<10;i2++)
					{
						*(&data1[i2])= *(&data2[daudocao+1+i2]);
						if(*(&data1[i2])=='.')
						{
							count++;
							if(count==1)
							{
							 daucham=i2;
							}
						}
						if(*(&data1[i2])==',')
						{
							count++;
							if(count==2)
							{
							 cuoidocao=i2;
							}
						}
					}
					count=0;
					if(*(&data1[0])=='-')
					{
						for(i2=1;i2<daucham;i2++)
						{
							docao += (*(&data1[i2])-0x30)*pow(10,daucham-i2-1);
						}
						for(i2=daucham+1;i2<cuoidocao;i2++)
						{
							docao += (*(&data1[i2])-0x30)* pow(10,daucham-i2);
						}
						docao *=-1;
					}
					else
					{
						for(i2=0;i2<daucham;i2++)
						{
							docao+= (*(&data1[i2])-0x30)*pow(10,(daucham-i2-1));
						}
						for(i2=daucham+1;i2<cuoidocao;i2++)
						{
							docao += (*(&data1[i2])-0x30)*pow(10,daucham-i2);
						}
					}
				}
			}
		}
	}
	return docao;
}

float gps_vantocthang (uint8_t *data1,uint8_t *data2,int lenght)
{
	float vantoc=0;
	int i=0,i2=0,demphay=0,dauvantoc=0,cuoivantoc=0,daucham=0,count=0;
	for(i=0;i<lenght;i++)
	{
		if(*(&data2[i])==',')
		{
			demphay++;
			if(demphay==7)
			{
				dauvantoc=i;
				if(*(&data2[dauvantoc+1])==',')
				{
					vantoc=0;
				}
				else
				{
					for(i2=0;i2<10;i2++)
					{
						*(&data1[i2])= *(&data2[dauvantoc+1+i2]);
						if(*(&data1[i2])=='.')
						{
							count++;
							if(count==1)
							{
							 daucham=i2;
							}
						}
						if(*(&data1[i2])==',')
						{
							count++;
							if(count==2)
							{
							 cuoivantoc=i2;
							}
						}
					}
					count=0;
					for(i2=0;i2<daucham;i2++)
					{
					 vantoc += (*(&data1[i2])- 0x30)* pow(10,daucham-i2-1);
					}
					for(i2=daucham+1;i2<cuoivantoc;i2++)
					{
						vantoc += (*(&data1[i2])- 0x30)* pow(10,daucham-i2);
					}
					
				}
			}
		}
	}
	return vantoc;
}

	






