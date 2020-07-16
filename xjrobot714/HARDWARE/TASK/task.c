#include "task.h"
#include "led.h"
#include "timer.h"
#include "can.h"
#include "usart.h"
#include "can_drive.h"
#include "dataprocess.h"
#include "engine.h"
#include "math.h"
#include "battery.h"

//此数据需要通过外部传感器读取
//初始化惯导数据 Roll pitch  Yaw
float Pitch=0.00,Yaw=0.00,Roll=0.00;
u8 emginestarstopstate=0xFF;

uint8_t underplatestate=0x00;


//创建任务
void creatTask(void)
{
  	LEDTASK();                //任务：状态指示灯
	  USART1_ReceiveData();     //任务：PC数据通过串口1接收
	  USART1Data_Process();     //任务：串口1数据处理
  	MotorPositionRequest();   //任务：请求获得电机位置信息
  	Odeometer();              //任务：里程计数据
    PS2DataRecrive();         //任务：解析Ps2手柄
//	Massage_Timing_To_Pc();   //任务：信号处理单元toPC
  	PS2ControlRun(PS2KEY);    //任务：遥控器控制运动
//	EnginekeepRun();          //任务：保持发动机控制器处于运动状态
//	engineStartStop(emginestarstopstate); // 任务： 发送机启动停止控制
//  PS2ControlUnderPlate(PS2KEY); 
//  UnderPlantControl();
//	Battery_Data_Capture();    //任务: 电池信息
//	Carsport_state();//车状态，是否保持前进
//motor_State();//电机是否连接，且状态良好
//	time_test1();
}




//任务：状态指示灯
void LEDTASK(void)
{
	//定时器是一个以10us为单位的计数器
	if(ledCount>LEDRATE/2)
		{
			LED0(1);
		}
		else
		{
			LED0(0);
		}
	if(ledCount>LEDRATE)
				ledCount=0;	
}

//任务：解析Ps2手柄
u8 PS2KEY=0;
void PS2DataRecrive(void)
{
	int i=0,t=0;
	if(ps2RecCount>PS2RECTIME)
	if(Ps2ReceiveCount>0)
	{    
		ps2RecCount=0;
		for(i=0;i<7;i++)
		{
			if((Ps2Receive[i]==0xCC)&&(Ps2Receive[i+1]==0x33)&&(Ps2Receive[i+3]==0xFF))
			{
				PS2KEY=Ps2Receive[i+2];
				Ps2ReceiveCount=0;
				for(t=0;t<10;t++)
				{
					Ps2Receive[t]=0;
				}
			}
		}
	}
}

//任务：请求获得电机位置信息
void MotorPositionRequest(void)
{
	if(positionRequest>POSITIONREQUEST)
	{
		Motor_Position_Request(Motor1_Drive_ID);  //12驱动器在同一侧   
		Motor_Position_Request(Motor2_Drive_ID);  //12驱动器在同一侧 		
		Motor_Position_Request(Motor3_Drive_ID);  //12驱动器在同一侧 
		Motor_Position_Request(Motor4_Drive_ID);  //12驱动器在同一侧 
		positionRequest=0;
	}
}


//计算里程计
int32_t Pre_Motor1EncodeCount=0;
int32_t Pre_Motor4EncodeCount=0;
int32_t Pre_Motor2EncodeCount=0;
int32_t Pre_Motor3EncodeCount=0;

volatile float X=0,Y=0,Distance_L=0,Pre_Distance_L=0,Distance_check=0,Distance_Lpp=0;
volatile float Angle=0;
volatile float SpeedV_V=0,SpeedV_Ang=0,Pre_Value_X=0,Pre_Value_Y=0;

float Yaw_Angle=0,Pre_Yaw_Angle=0,Yaw_Angle_Diff=0;      // 类似于航向角
volatile float Gyration_radius;   //旋转半径
	 
float wheel_interval= 1076.0f;// 两轮轴距
float multiplier=4.0f;           //倍频数
float deceleration_ratio=30.0f;  //减速比
float wheel_diameter=330.2f;     //轮子直径，单位mm 13英寸 33.02cm
float line_number=2500.0f;       //码盘线数 
float pi_1_2=1.570796f;          //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;          //π*3/2
float pi_2_1=6.283186f;          //π*2
//float dt=0.122f;                 //采样时间间隔100ms 
float dt=0.10f;
float sin_=0;        //角度计算值
float cos_=0;
float delta_distance=0,delta_oriention=0;   //采样时间内走的距离
float yaw_number=1.0f;//改直行会出错
float distance_sum=0,distance_diff=0;
float Yaw_Angle_1=0; //dt时间内方向变化值

float const_frame=0,const_angle=0,const_speed=0; //速度m/s转编码值速度;
u8 once=1,iii=3;

void Odeometer(void)
{
	 int32_t Motor1encodeCountdiffvalue=0;
	 int32_t Motor2encodeCountdiffvalue=0;
	 int32_t Motor3encodeCountdiffvalue=0;
	 int32_t Motor4encodeCountdiffvalue=0;	 
	 int32_t right=0;
   int32_t left=0;	
	 if(once)  //常数只计算一次
    {
		const_frame=car_message.wheel_diameter*pi/(car_message.line_number*car_message.multiplier*car_message.deceleration_ratio);//pi*d/线数/预分频/减速比   / = 一个编码器值为多长 
		const_angle=const_frame/car_message.wheel_interval;//角度=上式/轮距	   // 一般在小于5度的时候   角度值=sin(角度)=tan(角度)=x/y;
		const_speed=10.0f*car_message.line_number*car_message.multiplier*car_message.deceleration_ratio/pi/car_message.wheel_diameter;//1mm/pi/d*2500*4*30*10 10分辨率		
    once=0;			
    }		
	 if(OdomCount>OdomTime)
	 {
		  Motor1encodeCountdiffvalue=Motor1EncodeCount-Pre_Motor1EncodeCount;
	    Motor2encodeCountdiffvalue=Motor2EncodeCount-Pre_Motor2EncodeCount;
	    Motor3encodeCountdiffvalue=Motor3EncodeCount-Pre_Motor3EncodeCount;
	    Motor4encodeCountdiffvalue=Motor4EncodeCount-Pre_Motor4EncodeCount;
		  //当编码器启动时，发生跳变时，值为0；3000rpm，线数10000 0.1s=50000线
		  if(Motor1encodeCountdiffvalue>50000&&Motor1encodeCountdiffvalue<-50000&&Motor2encodeCountdiffvalue>50000&&Motor2encodeCountdiffvalue&&
				 Motor3encodeCountdiffvalue>50000&&Motor3encodeCountdiffvalue&&Motor4encodeCountdiffvalue>50000&&Motor4encodeCountdiffvalue<-50000)
			{
			Motor1encodeCountdiffvalue=0;
			Motor2encodeCountdiffvalue=0;
			Motor3encodeCountdiffvalue=0;
			Motor4encodeCountdiffvalue=0;
			}
		 // 同一侧两个编码器平均值
			right=(Motor1encodeCountdiffvalue+Motor2encodeCountdiffvalue)/2.0f;// 1、2为右侧向前值为正
			left=-(Motor4encodeCountdiffvalue+Motor3encodeCountdiffvalue)/2.0f;//3、4为左侧向前值为负
			distance_sum = 0.5f*(right+left);//短时间内，前进距离两侧速度和
			distance_diff = right-left;//短时间内，转弯角度为两侧速度差
			 
			Yaw_Angle_Diff = distance_diff * const_angle;// 采样期间走的角度
			Yaw_Angle=Yaw_Angle+Yaw_Angle_Diff;//计算里程计方向角
			Yaw_Angle_1 = Yaw_Angle + 0.5f * Yaw_Angle_Diff;//里程计方向角数据位数变化，用于三角函数计算

      sin_ = sin(Yaw_Angle_1);//计算出采样时间内的y坐标
      cos_ = cos(Yaw_Angle_1);//计算出采样时间内的x坐标	 
			X=X+distance_sum*cos_ *const_frame;   //距离*Yaw（偏角） 解析X坐标
			Y=Y+distance_sum*sin_*const_frame;
		 
		  // 当前瞬时线速度  w=th/t
		  SpeedV_Ang=Yaw_Angle_Diff/dt;    //角速度		 
		 	SpeedV_V=(distance_sum*const_frame)/dt;    //速度=位移/时间； 时间=定时间*Timer     
			Pre_Value_X=X;
			Pre_Value_Y=Y;
			if(Yaw_Angle > pi)
			{
			 Yaw_Angle -= pi_2_1;//减2pi
			}
			else
			{
			 if(Yaw_Angle < -pi)
				{
				 Yaw_Angle+=pi_2_1;//加2pi
				}
			}			
			OdomCount=0;		 
			WriteP_Data_Fusion->Detial.Head1=0xCC;
			WriteP_Data_Fusion->Detial.Head2=0x33;
			WriteP_Data_Fusion->Detial.Lenghth=0x16;  //22个字节
			WriteP_Data_Fusion->Detial.Order=0x11;
			WriteP_Data_Fusion->Detial.DistanceX=(int32_t)(X);
			WriteP_Data_Fusion->Detial.DistanceY=(int32_t)(Y);
			WriteP_Data_Fusion->Detial.Angular=(int32_t)(Yaw_Angle*1000);
			WriteP_Data_Fusion->Detial.SpeedV=(int32_t)(SpeedV_V);
			WriteP_Data_Fusion->Detial.SpeedAng=(int32_t)(SpeedV_Ang*1000);
			WriteP_Data_Fusion->Detial.SumCheck=0x00;								 
			for(iii=3;iii<25;iii++)
			{
				WriteP_Data_Fusion->Detial.SumCheck+=WriteP_Data_Fusion->bytes[iii];
			}
			Pre_Motor1EncodeCount=Motor1EncodeCount;
			Pre_Motor2EncodeCount=Motor2EncodeCount;
			Pre_Motor3EncodeCount=Motor3EncodeCount;
			Pre_Motor4EncodeCount=Motor4EncodeCount;
			Data_Fusion_Fre++;//100ms加一	
	 }	 
	 if(DataFrequency==0)
	 {
			USART1_Transmi_Data(WriteP_Data_Fusion->bytes,sizeof(WriteP_Data_Fusion->bytes));
			DataFrequency=255;
	 }
	else if((Data_Fusion_Fre>=(100000/10/OdomTime*DataFrequency))&&(DataFrequency!=255) )                //数据融合后坐标角度上传
	 {
			USART1_Transmi_Data(WriteP_Data_Fusion->bytes,sizeof(WriteP_Data_Fusion->bytes));
			Data_Fusion_Fre=0;	
	 }
 }

 
//串口1数据接受
void USART1_ReceiveData(void)
{
	if(USART1_Rx_Time_count>=Max_Rx_Time)                               //定时器计时大于设置的接收时间段
	{	
		USART1_Rx_Time_count = 0;		
		if(Rx_count_UART1)
		{
			Usart1RxLength[Usart1Rxend] = Rx_count_UART1;
			Rx_count_UART1 = 0;
			Usart1Rxend = (Usart1Rxend+1)%USART_RX_ARRAY_NUM;		
			if(Usart1RxNum < USART_RX_ARRAY_NUM)
			{
				Usart1RxNum++;
			}
			else
			{
				Usart1RxHead = (Usart1RxHead+1)%USART_RX_ARRAY_NUM;
			}
		}
	}
}

//串口1数据处理 
void USART1Data_Process(void)
	
{
	 if(Usart1RxNum)
	{
		  Usart1_Data_Analyse(USART1_RX_BUF[Usart1RxHead],USART1_RX_BUF[Usart1RxHead][2]+3);
	    Usart1RxHead = (Usart1RxHead+1)%USART_RX_ARRAY_NUM;
		  Usart1RxNum--;
	}
}


//解析遥控器数据
u8 ps2DataFlg=0xFF;
u8 state_yksp=0,state_pcsp=0;
int speedCoefficient=1;
int32_t speed1=0,speed2=0,speed3=0,speed4=0;
void PS2ControlRun(u8 ps2data)
{
	if(ps2data!=ps2DataFlg)
	{		
		  state_yksp=1;
			switch(ps2data)
			{
			 case 0x00:  //停 
				  state_yksp=0;
					speed1=0;
					speed2=0;
					speed3=0;
					speed4=0;
					break;
			 case 0x05:   //前
					CarsportR(-1,300+100*speedCoefficient);
			    break;
			 case 0x07:   //后
			    CarsportR(-1,-300-100*speedCoefficient);
			    break;
			 case 0x08:   //左转
					CarsportR(0,300+100*speedCoefficient);	
					break;
			 case 0x06:   //右转
					CarsportR(0,-300-100*speedCoefficient);			 
					break;	
									/**********发动机功能**********************/   
									 // 按时定为发动机启动 0x0E 没有确
									// 若不成功需要测试确定具体键值是多少 遥控器上 圆形
			 case 0x0E:
					emginestarstopstate=0X0E;
					//CarsportR(1200,200+100*speedCoefficient);
					break;
			    // 暂时定位发动机 灭火 0x10   遥控器 方框
			 case 0x10:
					//CarsportR(1200,-200-100*speedCoefficient);
					emginestarstopstate=0X10;
					break;
			
		                 /**********高低速功能**********************/    				 
			 case 0x0D:   //高速
					 {
					  if(speedCoefficient>3)
							speedCoefficient=4;
						else
							speedCoefficient=speedCoefficient+1;
						}
						break;
			 case 0x0F:   //低速
						{
							if(speedCoefficient<2)
							 speedCoefficient=1;
							else
							 speedCoefficient=speedCoefficient-1;
						}
			      break;
			 default:
					speed1=0;
					speed2=0;
					speed3=0;
					speed4=0;
			    state_yksp=0;
			    break;
				}
			 	Motor_Velocity(Motor1_Drive_ID,(uint32_t)(speed1));
				Motor_Velocity(Motor2_Drive_ID,(uint32_t)(speed2));
				Motor_Velocity(Motor3_Drive_ID,(uint32_t)(speed3));
				Motor_Velocity(Motor4_Drive_ID,(uint32_t)(speed4));
				ps2DataFlg=ps2data;
	}
}



void PS2ControlUnderPlate(u8 ps2data)
{
		switch(ps2data)
		{
			/**********底盘升降功能**********************/  	
			 // 具体按键值需要具体测试
			 // 底盘上升
			 case 0x0B:   
				 underPlantUp();
				 break;
			 // 底盘下降
			 case 0x09:
				 underPlantDown();
				 break;
		}	
}

void CarsportR(float rad,float ver)//rad半径单位mm,ver单位mm
{			
	 if (rad<0)//直行
		 {
		 speed1=const_speed*ver;
		 speed2=const_speed*ver;
		 speed3=-const_speed*ver;
		 speed4=-const_speed*ver;
		 }
	 else if(rad==0)//原地
		 {
		 speed1=const_speed*ver;
		 speed2=const_speed*ver;
		 speed3=const_speed*ver;
		 speed4=const_speed*ver;
		 }
	else   //有半径
	  {
		 if(ver>0)
			{
		  speed1=const_speed*(1+wheel_interval/2/rad)*ver;
		  speed2=const_speed*(1+wheel_interval/2/rad)*ver;
		  speed3=-const_speed*(1-wheel_interval/2/rad)*ver;
		  speed4=-const_speed*(1-wheel_interval/2/rad)*ver;
			}
		 else if(ver<0)
			{
			 speed1=-const_speed*(1-wheel_interval/2/rad)*ver;
			 speed2=-const_speed*(1-wheel_interval/2/rad)*ver;
			 speed3=const_speed*(1+wheel_interval/2/rad)*ver;
			 speed4=const_speed*(1+wheel_interval/2/rad)*ver;
			}
	  }	
}


//下位机超过car_sportstate 5s未收到控制指令，停止运动，同时不影响遥控器控制
void Carsport_state(void)
{
  if(carsport_state>time_5s)
	{
	 state_pcsp=0;
	 carsport_state=0;
	 if((state_yksp==1||state_pcsp==1)==0)
	 {
		Motor_Velocity(Motor1_Drive_ID,(uint32_t)(0));
		Motor_Velocity(Motor2_Drive_ID,(uint32_t)(0));
		Motor_Velocity(Motor3_Drive_ID,(uint32_t)(0));
		Motor_Velocity(Motor4_Drive_ID,(uint32_t)(0));    		
	 }	
	}
}

void motor_State(void)//每隔300ms，测电机电流与电机状态
{
 if(motor_state>time_300ms)
 {
	motor_state=0;
  Motor_Current_Request(Motor1_Drive_ID);
  Motor_Current_Request(Motor2_Drive_ID);
	Motor_Current_Request(Motor3_Drive_ID);
	Motor_Current_Request(Motor4_Drive_ID);
	Motor_Error_query(Motor1_Drive_ID);
	Motor_Error_query(Motor2_Drive_ID);
	Motor_Error_query(Motor3_Drive_ID);
	Motor_Error_query(Motor4_Drive_ID);
	motor_state=0;
 }
}

CAR_id car_message;
void data_init(u8 id)
{
	switch(id)
	{
		case 0x01:  //框架车
		  car_message.id1=0x01;
		  car_message.id2=0xE8;
		  car_message.wheel_interval= 1076.0f;// 两轮轴距
			car_message.multiplier=4.0f;           //倍频数
			car_message.deceleration_ratio=30.0f;  //减速比
			car_message.wheel_diameter=330.2f;     //轮子直径，单位mm 13英寸 33.02cm
			car_message.line_number=2500.0f;       //码盘线数		  
		break;
		case 0x02:  //G33
			car_message.id1=0x01;
		  car_message.id2=0xE8;
		  car_message.wheel_interval= 1040.0f;// 两轮轴距
			car_message.multiplier=4.0f;           //倍频数
			car_message.deceleration_ratio=50.0f;  //减速比
			car_message.wheel_diameter=330.2f;     //轮子直径，单位mm 13英寸 33.02cm
			car_message.line_number=2500.0f;       //码盘线数
		break;
	}


}

