#include "battery.h"



//BatteryDataReceive[10];        
//u8 batteryDataReceiveCnt;        // 电池信息接收


u8 battery;
int ib=0,kb=0;
void  Battery_Data_Capture()
{
	 if(batteryDataReceiveCnt>0)
	 {
		for(ib=0;ib<27;ib++)
		{
			if((BatteryDataReceive[ib]==0x5B)&&(BatteryDataReceive[ib+1]==0x2B)&&(BatteryDataReceive[ib+4]==0x2E))
			{
				//battery=BatteryDataReceive[ib+2];
				battery=(BatteryDataReceive[ib+2]-'0')*0x0A+(BatteryDataReceive[ib+3]-'0');
				batteryDataReceiveCnt=0;
				for(kb=0;kb<27;kb++)
		    {
		     BatteryDataReceive[kb]=0;
	      }	
			}
		}		
	}

}

