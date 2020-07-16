#include "dataprocess.h"
#include "sys.h"
#include "usart.h"
#include "task.h"
#include "can.h"
#include "can_drive.h"
#include "timer.h"
#include "engine.h"
#include "battery.h"
//变量定义

uint8_t UnderPlantUpdownConState;
uint32_t underplantttime;
uint8_t DataFrequency;
uint8_t SpeedFrequency;
uint8_t IMU_DataFrequency;
uint8_t Car_Configuration_FRQ_1;
uint8_t Car_Configuration_FRQ_2;

uint32_t Car_Motion_status_Fre;
uint32_t Data_Fusion_Fre=0;
uint32_t IMU_Data_Fre;

//结构体定义
ControlDef Car_Move_Conrol;
ConfigDef Car_Configuration;
Return Massage_Return,motor_statusRet,underplate_return,engineSs_return;
Return_set Massage_Return_set;
Return Massage_Return,Error_Clear;
Return_Odom Message_Return_Odom;
Error_Massage OutPut_Error_Massage;
uint8_t7_union battery_return,oil_return;
Current_union Current_return;
Current_union *Send_Current=&Current_return;

//结构体指针
//Ultrasonic_Massage Car_Ultrasonic_Massage[2];
//Ultrasonic_Massage *SendP_Ultrasonic=&Car_Ultrasonic_Massage[0],*WriteP_Ultrasonic=&Car_Ultrasonic_Massage[1];

Motion_status_Massage Car_Motion_status_Massage[2];
Motion_status_Massage *SendP_Car_Motion_status=&Car_Motion_status_Massage[0],*WriteP_Car_Motion_status=&Car_Motion_status_Massage[1];

Position Car_Position[2];
Position *SendP_Data_Fusion=&Car_Position[0],*WriteP_Data_Fusion=&Car_Position[1];

IMU_Data_inf IMU_Data_Detial[2];
IMU_Data_inf *SendP_Data_IMU=&IMU_Data_Detial[0],*WriteP_Data_IMU=&IMU_Data_Detial[1];


//数据出错返回上位机
void Output_Error(uint8_t error)
{
	OutPut_Error_Massage.detail.Head1=0xCC;
	OutPut_Error_Massage.detail.Head2=0x33;
	OutPut_Error_Massage.detail.Lenghth=0x03;
	OutPut_Error_Massage.detail.Order=0xFF;
	OutPut_Error_Massage.detail.Error=error;
	OutPut_Error_Massage.detail.SumCheck=(uint8_t)(OutPut_Error_Massage.detail.Order+OutPut_Error_Massage.detail.Error);
	//串口1数据发送
	USART1_Transmi_Data(OutPut_Error_Massage.bytes,sizeof(OutPut_Error_Massage));
}



int Check_Sum(uint8_t pData[],int num)
{
	uint8_t Sum=0;
	int count=3;
	for(count=3;count<num-1;count++)
	{
		Sum=(uint8_t)(Sum+pData[count]);
	}
	if(pData[num-1]==Sum)
		return 1;
	else
		return 0;
}

//串口1数据接收解析
int16_t speedline=0;
int16_t speedangle=0;
int dirline=1,dirangle=1;
int16_t speedlinedata=0,speedangledata=0;
int speedline_K=1500;   //转化为实际速度值  150--0.096m/s
int speedangle_K=150;   //转化为实际速度值   150 ---->  0.714rad/s
int32_t speed11=0,speed22=0,speed33=0,speed44=0;
void Usart1_Data_Analyse(uint8_t *Point, uint8_t num)	
{
   if(Point[0]==0xCC&&Point[1]==0x33)
	 {
				if(Check_Sum(Point,num)==1)
				{
					switch(Point[3])
					{
						case 0x01://运动控制指令
							//===============================================================================//
							//                小车运动控制                                                   //
							//===============================================================================//
							if(Point[8]==1)
							{
								//数据转换 收到的u8强制转成u16 将高位低位相加 Point[4]=低8位，Point[5]=高8位
								speedlinedata=(int16_t)(((uint16_t)Point[5]<<8)+(uint16_t)Point[4]);
								speedangledata=(int16_t)(((uint16_t)Point[7]<<8)+(uint16_t)Point[6]);
								state_pcsp=1;
								carsport_state=0;
								if (speedangledata==0)//直行
		              {
		               speed11=2891.97314*speedlinedata;
		               speed22=2891.97314*speedlinedata;
		               speed33=-2891.97314*speedlinedata;
		               speed44=-2891.97314*speedlinedata;
		               }
								 else if(speedlinedata==0)//原地
								   {
									 speed11=0.5f*2891.97314*speedangledata*wheel_interval/1000;//除2单侧
		               speed22=0.5f*2891.97314*speedangledata*wheel_interval/1000;
		               speed33=0.5f*2891.97314*speedangledata*wheel_interval/1000;
		               speed44=0.5f*2891.97314*speedangledata*wheel_interval/1000;
									 }
								 else
									 {
										 //const_speed
									 speed11=2891.97314*(1+wheel_interval/2*speedangledata/1000/speedlinedata)*speedlinedata;
									 speed22=2891.97314*(1+wheel_interval/2*speedangledata/1000/speedlinedata)*speedlinedata;
									 speed33=-2891.97314*(1-wheel_interval/2*speedangledata/1000/speedlinedata)*speedlinedata;
									 speed44=-2891.97314*(1-wheel_interval/2*speedangledata/1000/speedlinedata)*speedlinedata;
									 }							
							}
							else
							{	
						     //速度为0小车停止
               speed11=0;
							 speed22=0;
							 speed33=0;
							 speed44=0;
							}
							Motor_Velocity(Motor1_Drive_ID,(uint32_t)(speed11));
							Motor_Velocity(Motor2_Drive_ID,(uint32_t)(speed22));
							Motor_Velocity(Motor3_Drive_ID,(uint32_t)(speed33));
							Motor_Velocity(Motor4_Drive_ID,(uint32_t)(speed44));
						    //===============================给工控机反馈运动控制指令接收成功============================//
							Massage_Return.detail.Head1=0xCC;
							Massage_Return.detail.Head2=0x33;
							Massage_Return.detail.Lenghth=0x03;
							Massage_Return.detail.Order=0x01;
							Massage_Return.detail.Return_Data=0x41;
							Massage_Return.detail.SumCheck=(uint8_t)(Massage_Return.detail.Order+Massage_Return.detail.Return_Data);
						    //串口1发送
						  USART1_Transmi_Data(Massage_Return.bytes,sizeof(Massage_Return));			
							//================================end=============================================//                                         
					    break ;							
						case 0x02: //平台配置指令
						  break;
						case 0x10: //电机状态信息查询	
							motor_statusRet.detail.Head1=0xCC;
							motor_statusRet.detail.Head2=0x33;
							motor_statusRet.detail.Lenghth=0x03;
							motor_statusRet.detail.Order=0x10;
							motor_statusRet.detail.Return_Data=(Motor1Error|Motor2Error|Motor3Error|Motor4Error);//按位或|，逻辑或||
							motor_statusRet.detail.SumCheck=(uint8_t)(motor_statusRet.detail.Order+motor_statusRet.detail.Return_Data);
							USART1_Transmi_Data(motor_statusRet.bytes,sizeof(motor_statusRet));						
						  break;   
						case 0x11: // 信号处理单元 融入数据查询 即融合信息上传频率设定  
							DataFrequency=Point[4];   //信息频率上传确定  0为单次查询
						//	SpeedFrequency=Point[5];
							break;
						case 0x12://里程计清零指令							 
							X=0;
							Y=0;
							Yaw_Angle=0;
							//===============================给工控机反馈里程数据已经清零============================//
							Massage_Return.detail.Head1=0xCC;
							Massage_Return.detail.Head2=0x33;
							Massage_Return.detail.Lenghth=0x03;
							Massage_Return.detail.Order=0x12;
							Massage_Return.detail.Return_Data=0x43;
							Massage_Return.detail.SumCheck=(uint8_t)(Massage_Return.detail.Order+Massage_Return.detail.Return_Data);
						  //串口1发送
						  USART1_Transmi_Data(Massage_Return.bytes,sizeof(Massage_Return));						 			
			           //================================end=============================================//						 
						case 0x13:
						  break;
						case 0x31:      //IMU数据查询
							IMU_DataFrequency=Point[4];
							break;						 
            case 0x32:    //编码器数据进行校正
							if(Point[4]==0x00)  //记录编码器数据
							{																
								//===============================给工控机反馈里程数据已经清零============================//
								Massage_Return_set.detail.Head1=0xCC;
								Massage_Return_set.detail.Head2=0x33;
								Massage_Return_set.detail.Lenghth=0x03;
								Massage_Return_set.detail.Order=0x32;
								Massage_Return_set.detail.Return_Data=0x00;
								Massage_Return_set.detail.SumCheck=(uint8_t)(Massage_Return_set.detail.Order+Massage_Return_set.detail.Return_Data);
								//串口1发送数据
								USART1_Transmi_Data(Massage_Return_set.bytes,sizeof(Massage_Return_set));
								//HAL_UART_Transmit_IT(&huart1,Massage_Return_set.bytes,sizeof(Massage_Return_set));				
			         //================================end=============================================//								
							}
							else if(Point[4]==0x01) //记录编码器数据，计算对应的比例系数，存储在Flash中
							{
								//计算比例系数
								//Odom_Kp_Data.Kp=(0.1*Point[5])/(Distance_check-Distance_L_check);
								//const u8 TEXT_Buffer[]={Odom_Kp_Data.Odom_Kp_str.Kp1,Odom_Kp_Data.Odom_Kp_str.Kp2,Odom_Kp_Data.Odom_Kp_str.Kp3,Odom_Kp_Data.Odom_Kp_str.Kp4,0xFF-Odom_Kp_Data.Odom_Kp_str.Kp1,0xFF-Odom_Kp_Data.Odom_Kp_str.Kp2,0xFF-Odom_Kp_Data.Odom_Kp_str.Kp3,0xFF-Odom_Kp_Data.Odom_Kp_str.Kp4};
								//STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TEXT_Buffer,2);    //写入Flash								
								//KpReadFromFlash();
								//===============================给工控机反馈里程数据已经清零============================//
								Massage_Return_set.detail.Head1=0xCC;
								Massage_Return_set.detail.Head2=0x33;
								Massage_Return_set.detail.Lenghth=0x03;
								Massage_Return_set.detail.Order=0x32;
								Massage_Return_set.detail.Return_Data=0x01;
								Massage_Return_set.detail.SumCheck=(uint8_t)(Massage_Return_set.detail.Order+Massage_Return_set.detail.Return_Data);
								//串口1发送
								USART1_Transmi_Data(Massage_Return_set.bytes,sizeof(Massage_Return_set));								
								//HAL_UART_Transmit_IT(&huart1,Massage_Return_set.bytes,sizeof(Massage_Return_set));				
			         //================================end=============================================//
							}
						  break;
						case 0x33:
							if(Point[4]==0x00)  //记录编码器数据
							{
								/*Distance_L_check=Distance_check;
								//===============================给工控机反馈里程数据已经清零============================//
								//===============================给工控机反馈里程数据已经清零============================//
								Message_Return_Odom.detail.Head1=0xCC;
								Message_Return_Odom.detail.Head2=0x33;
								Message_Return_Odom.detail.Lenghth=0x04;
								Message_Return_Odom.detail.Order=0x33;
								Message_Return_Odom.detail.Return_Data=0x00;
								Message_Return_Odom.detail.Distance=0;
								Message_Return_Odom.detail.SumCheck=(uint8_t)(Message_Return_Odom.detail.Order+Message_Return_Odom.detail.Return_Data+Message_Return_Odom.detail.Distance);
								HAL_UART_Transmit_IT(&huart1,Message_Return_Odom.bytes,sizeof(Message_Return_Odom));	*/			
			                   //================================end=============================================//				
			                   //================================end=============================================//								
						  }
							else if(Point[4]==0x01) 
							{	
								//===============================给工控机反馈里程数据已经清零============================//
								Message_Return_Odom.detail.Head1=0xCC;
								Message_Return_Odom.detail.Head2=0x33;
								Message_Return_Odom.detail.Lenghth=0x04;
								Message_Return_Odom.detail.Order=0x33;
								Message_Return_Odom.detail.Return_Data=0x01;
								Message_Return_Odom.detail.Distance=0;
								Message_Return_Odom.detail.SumCheck=(uint8_t)(Message_Return_Odom.detail.Order+Message_Return_Odom.detail.Return_Data+Message_Return_Odom.detail.Distance);
								USART1_Transmi_Data(Message_Return_Odom.bytes,sizeof(Message_Return_Odom));								
								//HAL_UART_Transmit_IT(&huart1,Message_Return_Odom.bytes,sizeof(Message_Return_Odom));				
			                   //================================end=============================================//
							}					
						  break;							
						case 0x81:   // 发动机启停控制
							switch(Point[4])
							{
								// 启动命令 0x01 
								case 0x01:	
								 emginestarstopstate=0X0E;
								 engineSs_return.detail.Head1=0xCC;
								 engineSs_return.detail.Head2=0x33;
								 engineSs_return.detail.Lenghth=0x03;
								 engineSs_return.detail.Order=0x81;
								 engineSs_return.detail.Return_Data=0x41;
								 engineSs_return.detail.SumCheck=(uint8_t)(engineSs_return.detail.Order+engineSs_return.detail.Return_Data);
								  //串口1发送
								 USART1_Transmi_Data(engineSs_return.bytes,sizeof(engineSs_return));	
								 break;
								//停止命令
								case 0x02:
								//timedurcnt=0;
								 emginestarstopstate=0X10;
								 emginestarstopstate=0X0E;
								 engineSs_return.detail.Head1=0xCC;
								 engineSs_return.detail.Head2=0x33;
								 engineSs_return.detail.Lenghth=0x03;
								 engineSs_return.detail.Order=0x81;
								 engineSs_return.detail.Return_Data=0x41;
								 engineSs_return.detail.SumCheck=(uint8_t)(engineSs_return.detail.Order+engineSs_return.detail.Return_Data);
								  //串口1发送
								 USART1_Transmi_Data(engineSs_return.bytes,sizeof(engineSs_return));
									break;
							}
							break;
						case 0x82: //底盘升降
							  underplatestate=0xFF;
							  switch(Point[4])
								{
									case 0x01:
									UnderPlantUpdownConState=0x01;
									underplantttime=100;
									underplantsendState=0x00;
									underplate_return.detail.Head1=0xCC;
								  underplate_return.detail.Head2=0x33;
								  underplate_return.detail.Lenghth=0x03;
								  underplate_return.detail.Order=0x82;
								  underplate_return.detail.Return_Data=0x41;
								  underplate_return.detail.SumCheck=(uint8_t)(underplate_return.detail.Order+underplate_return.detail.Return_Data);
									//串口1发送
								  USART1_Transmi_Data(underplate_return.bytes,sizeof(underplate_return));									
									break;
									case 0x02:
									UnderPlantUpdownConState=0x01;
									underplantttime=50;
									underplantsendState=0x00;
									underplate_return.detail.Head1=0xCC;
								  underplate_return.detail.Head2=0x33;
								  underplate_return.detail.Lenghth=0x03;
								  underplate_return.detail.Order=0x82;
								  underplate_return.detail.Return_Data=0x41;
								  underplate_return.detail.SumCheck=(uint8_t)(underplate_return.detail.Order+underplate_return.detail.Return_Data);
									//串口1发送
								  USART1_Transmi_Data(underplate_return.bytes,sizeof(underplate_return));
									break;
									case 0x03:
									UnderPlantUpdownConState=0x02;
									underplantttime=50;
									underplantsendState=0x00;
									underplate_return.detail.Head1=0xCC;
								  underplate_return.detail.Head2=0x33;
								  underplate_return.detail.Lenghth=0x03;
								  underplate_return.detail.Order=0x82;
								  underplate_return.detail.Return_Data=0x41;
								  underplate_return.detail.SumCheck=(uint8_t)(underplate_return.detail.Order+underplate_return.detail.Return_Data);
									//串口1发送
								  USART1_Transmi_Data(underplate_return.bytes,sizeof(underplate_return));
									break;
									case 0x04:
									UnderPlantUpdownConState=0x02;
									underplantttime=100;
									underplantsendState=0x00;
									underplate_return.detail.Head1=0xCC;
								  underplate_return.detail.Head2=0x33;
								  underplate_return.detail.Lenghth=0x03;
								  underplate_return.detail.Order=0x82;
								  underplate_return.detail.Return_Data=0x41;
								  underplate_return.detail.SumCheck=(uint8_t)(underplate_return.detail.Order+underplate_return.detail.Return_Data);
									//串口1发送
								  USART1_Transmi_Data(underplate_return.bytes,sizeof(underplate_return));
									break;
								}
								break;
						  case 0x90: //状态查询
								 switch(Point[4])
								 {
								 case 0x01:
									 battery_return.detail.Head1=0xCC;
									 battery_return.detail.Head2=0x33;
									 battery_return.detail.Lenghth=0x03;
									 battery_return.detail.Order=0x90;
									 battery_return.detail.Order1=0x01;
									 battery_return.detail.Return_Data=battery;
									 battery_return.detail.SumCheck=(uint8_t)(battery_return.detail.Order+battery_return.detail.Order1+battery_return.detail.Return_Data);
									 USART1_Transmi_Data(battery_return.bytes,sizeof(battery_return));
								   break;
								 case 0x02:
									 oil_return.detail.Head1=0xCC;
									 oil_return.detail.Head2=0x33;
									 oil_return.detail.Lenghth=0x03;
									 oil_return.detail.Order=0x90;
									 oil_return.detail.Order1=0x02;
									 oil_return.detail.Return_Data=oil;
									 oil_return.detail.SumCheck=(uint8_t)(oil_return.detail.Order+oil_return.detail.Order1+oil_return.detail.Return_Data);
									 USART1_Transmi_Data(oil_return.bytes,sizeof(oil_return));
									 break;
								 case 0x03:
									 oil=Point[5];
								   oil_return.detail.Head1=0xCC;
									 oil_return.detail.Head2=0x33;
									 oil_return.detail.Lenghth=0x03;
									 oil_return.detail.Order=0x90;
									 oil_return.detail.Order1=0x03;
									 oil_return.detail.Return_Data=oil;
									 oil_return.detail.SumCheck=(uint8_t)(oil_return.detail.Order+oil_return.detail.Order1+oil_return.detail.Return_Data);
									 USART1_Transmi_Data(oil_return.bytes,sizeof(oil_return));
								   break;
								 }
								break;
								case 0x91: //状态查询
									 Send_Current->Detial1.Head1=0xCC;
								   Send_Current->Detial1.Head2=0X33;
								   Send_Current->Detial1.Lenghth=0x12;
								   Send_Current->Detial1.Order=0x91;
								   Send_Current->Detial1.Return_Data1=Motor1Current;
								   Send_Current->Detial1.Return_Data2=Motor2Current;
								   Send_Current->Detial1.Return_Data3=Motor3Current;
								   Send_Current->Detial1.Return_Data4=Motor4Current;
								   Send_Current->Detial1.SumCheck=(Send_Current->Detial1.Order+Send_Current->Detial1.Return_Data1+Send_Current->Detial1.Return_Data2+Send_Current->Detial1.Return_Data3+Send_Current->Detial1.Return_Data4);
									 USART1_Transmi_Data(Send_Current->bytes,sizeof(Send_Current->bytes));
								   break;
								case 0x92:
									Motor_ErrorReset_Request(Motor1_Drive_ID);
									Motor_ErrorReset_Request(Motor2_Drive_ID);
									Motor_ErrorReset_Request(Motor3_Drive_ID);
									Motor_ErrorReset_Request(Motor4_Drive_ID);
									MOTOR_init();	
									Error_Clear.detail.Head1=0xCC;
									Error_Clear.detail.Head2=0X33;
									Error_Clear.detail.Lenghth=0x03;
									Error_Clear.detail.Order=0x92;
									Error_Clear.detail.Return_Data=0x41;
									Error_Clear.detail.SumCheck=(Error_Clear.detail.Order+Error_Clear.detail.Return_Data);
									USART1_Transmi_Data(Error_Clear.bytes,sizeof(Error_Clear.bytes));
									break;
						default :
							Output_Error(Undefined_Order_Error);           //无效指令							
					    break;
					}
				} // 校验核结束 
				else
				{
					Output_Error(Checksum_Error);
				}
	}//帧头校验结束
}


void Massage_Timing_To_Pc(void)
{
	Motion_status_Massage *TransP_Car_Motion_status;
//	Ultrasonic_Massage *TransP_Ultrasonic;
	IMU_Data_inf *TransP_IMU_Data;
	
//*********************************************END*****************************************************************//	
					WriteP_Car_Motion_status->detail.Head1=0xCC;
					WriteP_Car_Motion_status->detail.Head2=0x33;
					WriteP_Car_Motion_status->detail.Lenghth=0x0A;
					WriteP_Car_Motion_status->detail.Order=0x08;
					WriteP_Car_Motion_status->detail.SumCheck=(uint8_t)(WriteP_Car_Motion_status->detail.Order+WriteP_Car_Motion_status->detail.LED_1+  \
																											 WriteP_Car_Motion_status->detail.LED_2+WriteP_Car_Motion_status->detail.LED_3+  \
																											 WriteP_Car_Motion_status->detail.LED_4+WriteP_Car_Motion_status->detail.LED_5+  \
																							 WriteP_Car_Motion_status->detail.Charge_status+WriteP_Car_Motion_status->detail.Power+  \
																										WriteP_Car_Motion_status->detail.Error);
//*****************************************小车状态上传*********************************************************************//	
	if(Car_Configuration_FRQ_2==0)
	{
		TransP_Car_Motion_status=SendP_Car_Motion_status;
		SendP_Car_Motion_status=WriteP_Car_Motion_status;
		WriteP_Car_Motion_status=TransP_Car_Motion_status;
		
		USART1_Transmi_Data(SendP_Car_Motion_status->bytes,sizeof(SendP_Car_Motion_status->bytes));
		
	    Car_Configuration_FRQ_2=255;
	}
	else if((Car_Motion_status_Fre>(100000/Car_Configuration_FRQ_2)) &&(Car_Configuration_FRQ_2!=255)) //运动状态上传
	{	
		TransP_Car_Motion_status=SendP_Car_Motion_status;
		SendP_Car_Motion_status=WriteP_Car_Motion_status;
		WriteP_Car_Motion_status=TransP_Car_Motion_status;
		USART1_Transmi_Data(SendP_Car_Motion_status->bytes,sizeof(SendP_Car_Motion_status->bytes));
		Car_Motion_status_Fre=0;
	}
	
	//********************************************IMU数据信息上传******************************************************************//	
	
	if(IMU_DataFrequency==0)
	{
		WriteP_Data_IMU->Detial.Head1=0xCC;
		WriteP_Data_IMU->Detial.Head2=0x33;
		WriteP_Data_IMU->Detial.Lenghth=0x0D;
		WriteP_Data_IMU->Detial.Order=0x31;
		WriteP_Data_IMU->Detial.Roll =Roll;
		WriteP_Data_IMU->Detial.Pitch =Pitch;
		WriteP_Data_IMU->Detial.Yaw =Yaw;
		WriteP_Data_IMU->Detial.SumCheck=(WriteP_Data_IMU->Detial.Order+ \
																			WriteP_Data_IMU->Detial.Roll+ \
																			WriteP_Data_IMU->Detial.Pitch+ \
																			WriteP_Data_IMU->Detial.Yaw);
		
		TransP_IMU_Data=SendP_Data_IMU;
		SendP_Data_IMU=WriteP_Data_IMU;
		WriteP_Data_IMU=TransP_IMU_Data;
		USART1_Transmi_Data(SendP_Data_IMU->bytes,sizeof(SendP_Data_IMU->bytes));
		IMU_DataFrequency=255;
	}
	else if((IMU_Data_Fre>(100000/IMU_DataFrequency))&&(IMU_DataFrequency!=255) )                //数据融合后坐标角度上传   IMU_Data_Fre=10us一次
	{
		WriteP_Data_IMU->Detial.Head1=0xCC;
		WriteP_Data_IMU->Detial.Head2=0x33;
		WriteP_Data_IMU->Detial.Lenghth=0x0D;
		WriteP_Data_IMU->Detial.Order=0x31;
		WriteP_Data_IMU->Detial.Roll =Roll;
		WriteP_Data_IMU->Detial.Pitch =Pitch;
		WriteP_Data_IMU->Detial.Yaw =Yaw;
		WriteP_Data_IMU->Detial.SumCheck=(WriteP_Data_IMU->Detial.Order+ \
																			WriteP_Data_IMU->Detial.Roll+ \
																			WriteP_Data_IMU->Detial.Pitch+ \
																			WriteP_Data_IMU->Detial.Yaw);
		TransP_IMU_Data=SendP_Data_IMU;
		SendP_Data_IMU=WriteP_Data_IMU;
		WriteP_Data_IMU=TransP_IMU_Data;
		USART1_Transmi_Data(SendP_Data_IMU->bytes,sizeof(SendP_Data_IMU->bytes));
	  IMU_Data_Fre=0;	
	}
}

