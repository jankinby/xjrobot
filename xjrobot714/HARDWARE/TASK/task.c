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

//��������Ҫͨ���ⲿ��������ȡ
//��ʼ���ߵ����� Roll pitch  Yaw
float Pitch=0.00,Yaw=0.00,Roll=0.00;
u8 emginestarstopstate=0xFF;

uint8_t underplatestate=0x00;


//��������
void creatTask(void)
{
  	LEDTASK();                //����״ָ̬ʾ��
	  USART1_ReceiveData();     //����PC����ͨ������1����
	  USART1Data_Process();     //���񣺴���1���ݴ���
  	MotorPositionRequest();   //���������õ��λ����Ϣ
  	Odeometer();              //������̼�����
    PS2DataRecrive();         //���񣺽���Ps2�ֱ�
//	Massage_Timing_To_Pc();   //�����źŴ���ԪtoPC
  	PS2ControlRun(PS2KEY);    //����ң���������˶�
//	EnginekeepRun();          //���񣺱��ַ����������������˶�״̬
//	engineStartStop(emginestarstopstate); // ���� ���ͻ�����ֹͣ����
//  PS2ControlUnderPlate(PS2KEY); 
//  UnderPlantControl();
//	Battery_Data_Capture();    //����: �����Ϣ
//	Carsport_state();//��״̬���Ƿ񱣳�ǰ��
//motor_State();//����Ƿ����ӣ���״̬����
//	time_test1();
}




//����״ָ̬ʾ��
void LEDTASK(void)
{
	//��ʱ����һ����10usΪ��λ�ļ�����
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

//���񣺽���Ps2�ֱ�
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

//���������õ��λ����Ϣ
void MotorPositionRequest(void)
{
	if(positionRequest>POSITIONREQUEST)
	{
		Motor_Position_Request(Motor1_Drive_ID);  //12��������ͬһ��   
		Motor_Position_Request(Motor2_Drive_ID);  //12��������ͬһ�� 		
		Motor_Position_Request(Motor3_Drive_ID);  //12��������ͬһ�� 
		Motor_Position_Request(Motor4_Drive_ID);  //12��������ͬһ�� 
		positionRequest=0;
	}
}


//������̼�
int32_t Pre_Motor1EncodeCount=0;
int32_t Pre_Motor4EncodeCount=0;
int32_t Pre_Motor2EncodeCount=0;
int32_t Pre_Motor3EncodeCount=0;

volatile float X=0,Y=0,Distance_L=0,Pre_Distance_L=0,Distance_check=0,Distance_Lpp=0;
volatile float Angle=0;
volatile float SpeedV_V=0,SpeedV_Ang=0,Pre_Value_X=0,Pre_Value_Y=0;

float Yaw_Angle=0,Pre_Yaw_Angle=0,Yaw_Angle_Diff=0;      // �����ں����
volatile float Gyration_radius;   //��ת�뾶
	 
float wheel_interval= 1076.0f;// �������
float multiplier=4.0f;           //��Ƶ��
float deceleration_ratio=30.0f;  //���ٱ�
float wheel_diameter=330.2f;     //����ֱ������λmm 13Ӣ�� 33.02cm
float line_number=2500.0f;       //�������� 
float pi_1_2=1.570796f;          //��/2
float pi=3.141593f;              //��
float pi_3_2=4.712389f;          //��*3/2
float pi_2_1=6.283186f;          //��*2
//float dt=0.122f;                 //����ʱ����100ms 
float dt=0.10f;
float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;
float delta_distance=0,delta_oriention=0;   //����ʱ�����ߵľ���
float yaw_number=1.0f;//��ֱ�л����
float distance_sum=0,distance_diff=0;
float Yaw_Angle_1=0; //dtʱ���ڷ���仯ֵ

float const_frame=0,const_angle=0,const_speed=0; //�ٶ�m/sת����ֵ�ٶ�;
u8 once=1,iii=3;

void Odeometer(void)
{
	 int32_t Motor1encodeCountdiffvalue=0;
	 int32_t Motor2encodeCountdiffvalue=0;
	 int32_t Motor3encodeCountdiffvalue=0;
	 int32_t Motor4encodeCountdiffvalue=0;	 
	 int32_t right=0;
   int32_t left=0;	
	 if(once)  //����ֻ����һ��
    {
		const_frame=car_message.wheel_diameter*pi/(car_message.line_number*car_message.multiplier*car_message.deceleration_ratio);//pi*d/����/Ԥ��Ƶ/���ٱ�   / = һ��������ֵΪ�೤ 
		const_angle=const_frame/car_message.wheel_interval;//�Ƕ�=��ʽ/�־�	   // һ����С��5�ȵ�ʱ��   �Ƕ�ֵ=sin(�Ƕ�)=tan(�Ƕ�)=x/y;
		const_speed=10.0f*car_message.line_number*car_message.multiplier*car_message.deceleration_ratio/pi/car_message.wheel_diameter;//1mm/pi/d*2500*4*30*10 10�ֱ���		
    once=0;			
    }		
	 if(OdomCount>OdomTime)
	 {
		  Motor1encodeCountdiffvalue=Motor1EncodeCount-Pre_Motor1EncodeCount;
	    Motor2encodeCountdiffvalue=Motor2EncodeCount-Pre_Motor2EncodeCount;
	    Motor3encodeCountdiffvalue=Motor3EncodeCount-Pre_Motor3EncodeCount;
	    Motor4encodeCountdiffvalue=Motor4EncodeCount-Pre_Motor4EncodeCount;
		  //������������ʱ����������ʱ��ֵΪ0��3000rpm������10000 0.1s=50000��
		  if(Motor1encodeCountdiffvalue>50000&&Motor1encodeCountdiffvalue<-50000&&Motor2encodeCountdiffvalue>50000&&Motor2encodeCountdiffvalue&&
				 Motor3encodeCountdiffvalue>50000&&Motor3encodeCountdiffvalue&&Motor4encodeCountdiffvalue>50000&&Motor4encodeCountdiffvalue<-50000)
			{
			Motor1encodeCountdiffvalue=0;
			Motor2encodeCountdiffvalue=0;
			Motor3encodeCountdiffvalue=0;
			Motor4encodeCountdiffvalue=0;
			}
		 // ͬһ������������ƽ��ֵ
			right=(Motor1encodeCountdiffvalue+Motor2encodeCountdiffvalue)/2.0f;// 1��2Ϊ�Ҳ���ǰֵΪ��
			left=-(Motor4encodeCountdiffvalue+Motor3encodeCountdiffvalue)/2.0f;//3��4Ϊ�����ǰֵΪ��
			distance_sum = 0.5f*(right+left);//��ʱ���ڣ�ǰ�����������ٶȺ�
			distance_diff = right-left;//��ʱ���ڣ�ת��Ƕ�Ϊ�����ٶȲ�
			 
			Yaw_Angle_Diff = distance_diff * const_angle;// �����ڼ��ߵĽǶ�
			Yaw_Angle=Yaw_Angle+Yaw_Angle_Diff;//������̼Ʒ����
			Yaw_Angle_1 = Yaw_Angle + 0.5f * Yaw_Angle_Diff;//��̼Ʒ��������λ���仯���������Ǻ�������

      sin_ = sin(Yaw_Angle_1);//���������ʱ���ڵ�y����
      cos_ = cos(Yaw_Angle_1);//���������ʱ���ڵ�x����	 
			X=X+distance_sum*cos_ *const_frame;   //����*Yaw��ƫ�ǣ� ����X����
			Y=Y+distance_sum*sin_*const_frame;
		 
		  // ��ǰ˲ʱ���ٶ�  w=th/t
		  SpeedV_Ang=Yaw_Angle_Diff/dt;    //���ٶ�		 
		 	SpeedV_V=(distance_sum*const_frame)/dt;    //�ٶ�=λ��/ʱ�䣻 ʱ��=��ʱ��*Timer     
			Pre_Value_X=X;
			Pre_Value_Y=Y;
			if(Yaw_Angle > pi)
			{
			 Yaw_Angle -= pi_2_1;//��2pi
			}
			else
			{
			 if(Yaw_Angle < -pi)
				{
				 Yaw_Angle+=pi_2_1;//��2pi
				}
			}			
			OdomCount=0;		 
			WriteP_Data_Fusion->Detial.Head1=0xCC;
			WriteP_Data_Fusion->Detial.Head2=0x33;
			WriteP_Data_Fusion->Detial.Lenghth=0x16;  //22���ֽ�
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
			Data_Fusion_Fre++;//100ms��һ	
	 }	 
	 if(DataFrequency==0)
	 {
			USART1_Transmi_Data(WriteP_Data_Fusion->bytes,sizeof(WriteP_Data_Fusion->bytes));
			DataFrequency=255;
	 }
	else if((Data_Fusion_Fre>=(100000/10/OdomTime*DataFrequency))&&(DataFrequency!=255) )                //�����ںϺ�����Ƕ��ϴ�
	 {
			USART1_Transmi_Data(WriteP_Data_Fusion->bytes,sizeof(WriteP_Data_Fusion->bytes));
			Data_Fusion_Fre=0;	
	 }
 }

 
//����1���ݽ���
void USART1_ReceiveData(void)
{
	if(USART1_Rx_Time_count>=Max_Rx_Time)                               //��ʱ����ʱ�������õĽ���ʱ���
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

//����1���ݴ��� 
void USART1Data_Process(void)
	
{
	 if(Usart1RxNum)
	{
		  Usart1_Data_Analyse(USART1_RX_BUF[Usart1RxHead],USART1_RX_BUF[Usart1RxHead][2]+3);
	    Usart1RxHead = (Usart1RxHead+1)%USART_RX_ARRAY_NUM;
		  Usart1RxNum--;
	}
}


//����ң��������
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
			 case 0x00:  //ͣ 
				  state_yksp=0;
					speed1=0;
					speed2=0;
					speed3=0;
					speed4=0;
					break;
			 case 0x05:   //ǰ
					CarsportR(-1,300+100*speedCoefficient);
			    break;
			 case 0x07:   //��
			    CarsportR(-1,-300-100*speedCoefficient);
			    break;
			 case 0x08:   //��ת
					CarsportR(0,300+100*speedCoefficient);	
					break;
			 case 0x06:   //��ת
					CarsportR(0,-300-100*speedCoefficient);			 
					break;	
									/**********����������**********************/   
									 // ��ʱ��Ϊ���������� 0x0E û��ȷ
									// �����ɹ���Ҫ����ȷ�������ֵ�Ƕ��� ң������ Բ��
			 case 0x0E:
					emginestarstopstate=0X0E;
					//CarsportR(1200,200+100*speedCoefficient);
					break;
			    // ��ʱ��λ������ ��� 0x10   ң���� ����
			 case 0x10:
					//CarsportR(1200,-200-100*speedCoefficient);
					emginestarstopstate=0X10;
					break;
			
		                 /**********�ߵ��ٹ���**********************/    				 
			 case 0x0D:   //����
					 {
					  if(speedCoefficient>3)
							speedCoefficient=4;
						else
							speedCoefficient=speedCoefficient+1;
						}
						break;
			 case 0x0F:   //����
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
			/**********������������**********************/  	
			 // ���尴��ֵ��Ҫ�������
			 // ��������
			 case 0x0B:   
				 underPlantUp();
				 break;
			 // �����½�
			 case 0x09:
				 underPlantDown();
				 break;
		}	
}

void CarsportR(float rad,float ver)//rad�뾶��λmm,ver��λmm
{			
	 if (rad<0)//ֱ��
		 {
		 speed1=const_speed*ver;
		 speed2=const_speed*ver;
		 speed3=-const_speed*ver;
		 speed4=-const_speed*ver;
		 }
	 else if(rad==0)//ԭ��
		 {
		 speed1=const_speed*ver;
		 speed2=const_speed*ver;
		 speed3=const_speed*ver;
		 speed4=const_speed*ver;
		 }
	else   //�а뾶
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


//��λ������car_sportstate 5sδ�յ�����ָ�ֹͣ�˶���ͬʱ��Ӱ��ң��������
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

void motor_State(void)//ÿ��300ms��������������״̬
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
		case 0x01:  //��ܳ�
		  car_message.id1=0x01;
		  car_message.id2=0xE8;
		  car_message.wheel_interval= 1076.0f;// �������
			car_message.multiplier=4.0f;           //��Ƶ��
			car_message.deceleration_ratio=30.0f;  //���ٱ�
			car_message.wheel_diameter=330.2f;     //����ֱ������λmm 13Ӣ�� 33.02cm
			car_message.line_number=2500.0f;       //��������		  
		break;
		case 0x02:  //G33
			car_message.id1=0x01;
		  car_message.id2=0xE8;
		  car_message.wheel_interval= 1040.0f;// �������
			car_message.multiplier=4.0f;           //��Ƶ��
			car_message.deceleration_ratio=50.0f;  //���ٱ�
			car_message.wheel_diameter=330.2f;     //����ֱ������λmm 13Ӣ�� 33.02cm
			car_message.line_number=2500.0f;       //��������
		break;
	}


}

