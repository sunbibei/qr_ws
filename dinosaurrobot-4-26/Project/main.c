#include "main.h"

#define kneecCoder ADC_Channel_4
#define hipCoder  ADC_Channel_5
#define yawCoder  ADC_Channel_6

u16 timeCountMs = 0,iwdgFeedTime = 0,ctlPeroidTime = 0;
u8 can1SendPeriod = 0,driverCheckPeriod = 0;      //can1SendPeriod:关节角度发送周期=50hz
u8 count = 0;
u16 kneeAngle = 0,hipAngle = 0,yawAngle = 0,kneeAngleOffset = 0,hipAngleOffset = 0,yawAngleOffset = 0;


int main(void)
   {	
	SystemInit();	
	delayInit();	    		 //延时函数初始化
	jointCoderInit();			 //关节角度编码器初始化
	iwdgInit(4,625);			 //独立看门狗初始化，溢出时间：1s	
	 
	can1ModeInit(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);		//CAN1初始化正常模式,波特率500kbps
	can2ModeInit(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);		//CAN2初始化正常模式,波特率500kbps
	
	if (SysTick_Config(SystemCoreClock / 1000))    /* Setup SysTick Timer for 1 msec interrupts  */
	{  
		while(1);/* Capture error */
	}
	
	kneeAngleOffset  = getJointCoder(kneecCoder);		//查询膝关节电机初始角度
	hipAngleOffset   = getJointCoder(hipCoder);			//查询髋关节电机初始角度
	yawAngleOffset   = getJointCoder(yawCoder);			//查询yaw关节电机初始角度
	
	LED_Init();

	while(1)
	{	
		if(timeCountMs >= 500) //unit:ms,超时没有接受到命令，停止
		{
			iwdgFeedTime = 0;
			timeCountMs = 0;
			LED0=!LED0;
			IWDG_ReloadCounter(); 

		}		
		
		if(can1SendPeriod >= 50)				//以50Hz频率发送电机关节角度,速度
		{

			count = count + 1;
			switch(count)
			{				
				case 1:
					kneeAngle = getJointCoder(kneecCoder);
					kneeAngle = 360*kneeAngle/4096;
					jointMsgProcess(kneeAngle,KNEE_MOTOR);
					can1SendMsg(jointPositionMsg,5);				   		//发送膝关节角度
					can1SendMsg(jointSpeedMsg,5);				   		 	  //发送膝关节速度
				break;
				case 2:
					hipAngle  = getJointCoder(hipCoder); 
					hipAngle  = 360*hipAngle/4096; 
					jointMsgProcess(hipAngle,HIP_MOTOR);
					can1SendMsg(jointSpeedMsg,5);				   		 	//发送髋关节速度
					can1SendMsg(jointPositionMsg,5);				   	//发送髋关节角度
				break;
				case 3:
					yawAngle  = getJointCoder(yawCoder);
					yawAngle  =  360*yawAngle/4096;
					jointMsgProcess(yawAngle,YAW_MOTOR);
					can1SendMsg(jointPositionMsg,5);				   		//发送yaw关节角度
					can1SendMsg(jointSpeedMsg,5);				   		 	  //发送yaw关节速度
				break;
				case 4:
					count = 0;
				break;
				default:break;
			}			
		}
		/**************fix by wang****************/
    if(ctlPeroidTime > 20)
		{
		  ctlPeroidTime = 0;
			ControlProcess();//完成控制	
		}
			
			
	} 
} 

