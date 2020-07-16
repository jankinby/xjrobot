#include "engine.h"
#include "usart.h"
#include "time.h"
#include "task.h"
#include "dataprocess.h"
#include "timer.h"
EngineControl EngineControlData;

void EngineSettings(uint8_t  function,uint8_t  accelerator)
{
	uint16_t EngineControlSum=0x00;
	uint8_t i;
	EngineControlData.detail.Head1=car_message.id1;        // �̶�
	EngineControlData.detail.Head2=car_message.id2;        // �̶�    EF ΪG33 
	EngineControlData.detail.Reserved=0x00;     // �̶�
	EngineControlData.detail.Function=function;  // ���ܺ���
	EngineControlData.detail.MotorHReserved=0x00;
	EngineControlData.detail.MotorLReserved=0xFA;   // �̶�ֵ
	EngineControlData.detail.MotorReserved=0x00;
	EngineControlData.detail.Accelerator=accelerator;   // ���ţ�0-180��
	for(i=0;i<7;i++)
	{
		EngineControlSum+=EngineControlData.bytes[i];
	}
	EngineControlData.detail.SumCheckL=(uint8_t)EngineControlSum;  // ȡ��8λ
	EngineControlData.detail.SumCheckH=(uint8_t)(EngineControlSum>>8); // ȡ��8λ
	
	UART8_Transmi_Data(EngineControlData.bytes,sizeof(EngineControlData.bytes));
}


int  timedurcnt=0;
uint32_t engineStarcnt=0;
// 01 E5 00 80 00 FA 00 5A 02 BA  ����ָ�� ����λΪУ���
u8 starstate=0xff;
void EngineStar(void)
{   
	
	//   ����5s ��  5000/100=5
	  if(timedurcnt<50)
		{
			if(engineStarcnt>ENGINESTRCNT)
			{    
				 EngineSettings(0x80,0x5A);
				  engineStarcnt=0;
					
				  // 0x80 ����  5AΪ ����ת�� 
				 timedurcnt++;
			}
	  }else {
			timedurcnt=0;
			emginestarstopstate=0xff;
		}
	
}


// ֹͣ����
//01 E5 00 80 00 00 00 00 01 66 ָֹͣ�� ����λΪУ��� 
void EngineClose(void)
{   
	//   ����5s ��  6000/100=60
	  if(timedurcnt<60)
		{
			if(engineStarcnt>ENGINESTRCNT)
			{  
          EngineSettings(0x80,0x00);				
				  engineStarcnt=0;
				  timedurcnt++;
					
			}
	  }else{
			timedurcnt=0;
			emginestarstopstate=0xff;
		}
	
}
	



void engineStartStop(u8 state)
{
	//����
	if(state==0x0E)
	{
		EngineStar();
	}
	//�ر�
	if(state==0x10)
	{
		EngineClose();
	}
}


	
u8 enginestate=0;
u16 enginekeepRuntimcnt=0;
u32 enginekeeptime=0;
u8 oil=100;
void EnginekeepRun(void)
{
    if((enginestate>0xA7))
		{
			 if(enginekeepRuntimcnt>ENGINEKEEPRUNTIMECNT)
			 {
				 EngineSettings(0x00,0x5A);
			   enginekeepRuntimcnt=0;
			 }
			 if(enginekeeptime>ENGINEKEEPTIME)
			 {
				 oil=oil-1;
				 enginekeeptime=0;
			 }
		}
		
}





u16 engineRecCount=0;
u8 enginestat=0;
// 01 E5 00 00 00 FA 00 00 01 E0  ��ָ���ж�������  ����λΪУ��� 
//  ����¼D3Ϊ11��D4ֵ��δ����ʱΪA6/A7������ʱΪAE/AF
void engineDataRecrive(void)
{
	int i=0,t=0;
	if(engineRecCount>ENGINRECNT)
	if(engineReceiveCount>0)
	{    
		engineRecCount=0;
		for(i=0;i<5;i++)
		{
			if((engineReceive[i]==0x01)&&(engineReceive[i+1]==0xE8))
			{
				  switch (engineReceive[i+3])  // D3  ֵ   
					{
						case 0x11:
							 //Ҫ���в����ĺ��� 
						   // �ж�D4 ��ֵ  A6/A7���� δ����  AE/AF ���������� 
						   // ������������ 01 E5 0B 11 A6
						    //���������б�־
						    enginestate=engineReceive[i+4];
							break;
						case 0x00:	
							break;
						case 0x20:
							break;
						case 0x30:
							break;
						case 0x41:
							break;
						
					}
								
				engineReceiveCount=0;
				for(t=0;t<10;t++)
				{
					engineReceive[t]=0;
				}
			}
		}
	}
}
	

/**************************************************************/
// ������������
void underPlantUp(void)
{
	// bit2=1 ��  bit2=0  �޲���
	EngineSettings(0x04,0x00);
}

void underPlantDown(void)
{
		// bit3=1 �½�  bit3=0  �޲���
	EngineSettings(0x08,0x00);
}


// �½�Լ6s
u16 timedurcntupplant=0;
u16 underplantsendState=0x00;



void underPlantDownTime(uint32_t times)
{
	if(underplantsendState!=0xFE)
	{
		if(timedurcntupplant<times)
		{
			if(underplantimeCnt<UNDERPLANTSENDFRN)
			{
				// bit3=1 ��  bit3=0  �޲���
					EngineSettings(0x08,0x00);
					underplantimeCnt=0;
					timedurcntupplant++;
			}
		}else {
			timedurcntupplant=0;
			underplantsendState=0xFE;
			UnderPlantUpdownConState=0x00;
		}
	}
}


void underPlantUpTime(uint32_t times)
{
	if(underplantsendState!=0XFE)
	{
		if(timedurcntupplant<times)
		{
			if(underplantimeCnt<UNDERPLANTSENDFRN)
			{
				// bit2=1 ��  bit2=0  �޲���
					EngineSettings(0x04,0x00);
					underplantimeCnt=0;
					timedurcntupplant++;
			}
		}else {
			timedurcntupplant=0;
			underplantsendState=0XFE;
			UnderPlantUpdownConState=0x00;
		}
  }
}




void UnderPlantControl(void)
{
	if(UnderPlantUpdownConState==0x01)
	{
		underPlantDownTime(underplantttime);
	}
	if(UnderPlantUpdownConState==0x02)
	{
		underPlantUpTime(underplantttime);
	}
}
/*

void UnderPlantUpdownCon(uint8_t underplatestate)
{
	   if(underplatestate==0xFF)
		 {
			 switch (UnderPlantUpdownTimeConState)
			 {
				 case 0x01:
					 //�½���Լ6s
				   underplantsendState=0x00;
					 underPlantDownTime(600);
				 break;
				 case 0x02:
					 // ������Լ2 ��
				   underplantsendState=0x00;
				   underPlantUpTime(200);
					 
				 break;
				 case 0x03:
					 
				 break;
				 underplatestate=0x00;
			 }
		 }
	
}
*/


