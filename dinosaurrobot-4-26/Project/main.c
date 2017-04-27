#include "main.h"

#define kneecCoder ADC_Channel_4
#define hipCoder  ADC_Channel_5
#define yawCoder  ADC_Channel_6

u16 timeCountMs = 0,iwdgFeedTime = 0,ctlPeroidTime = 0;
u8 can1SendPeriod = 0,driverCheckPeriod = 0;      //can1SendPeriod:�ؽڽǶȷ�������=50hz
u8 count = 0;
u16 kneeAngle = 0,hipAngle = 0,yawAngle = 0,kneeAngleOffset = 0,hipAngleOffset = 0,yawAngleOffset = 0;


int main(void)
   {	
	SystemInit();	
	delayInit();	    		 //��ʱ������ʼ��
	jointCoderInit();			 //�ؽڽǶȱ�������ʼ��
	iwdgInit(4,625);			 //�������Ź���ʼ�������ʱ�䣺1s	
	 
	can1ModeInit(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);		//CAN1��ʼ������ģʽ,������500kbps
	can2ModeInit(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);		//CAN2��ʼ������ģʽ,������500kbps
	
	if (SysTick_Config(SystemCoreClock / 1000))    /* Setup SysTick Timer for 1 msec interrupts  */
	{  
		while(1);/* Capture error */
	}
	
	kneeAngleOffset  = getJointCoder(kneecCoder);		//��ѯϥ�ؽڵ����ʼ�Ƕ�
	hipAngleOffset   = getJointCoder(hipCoder);			//��ѯ�Źؽڵ����ʼ�Ƕ�
	yawAngleOffset   = getJointCoder(yawCoder);			//��ѯyaw�ؽڵ����ʼ�Ƕ�
	
	LED_Init();

	while(1)
	{	
		if(timeCountMs >= 500) //unit:ms,��ʱû�н��ܵ����ֹͣ
		{
			iwdgFeedTime = 0;
			timeCountMs = 0;
			LED0=!LED0;
			IWDG_ReloadCounter(); 

		}		
		
		if(can1SendPeriod >= 50)				//��50HzƵ�ʷ��͵���ؽڽǶ�,�ٶ�
		{

			count = count + 1;
			switch(count)
			{				
				case 1:
					kneeAngle = getJointCoder(kneecCoder);
					kneeAngle = 360*kneeAngle/4096;
					jointMsgProcess(kneeAngle,KNEE_MOTOR);
					can1SendMsg(jointPositionMsg,5);				   		//����ϥ�ؽڽǶ�
					can1SendMsg(jointSpeedMsg,5);				   		 	  //����ϥ�ؽ��ٶ�
				break;
				case 2:
					hipAngle  = getJointCoder(hipCoder); 
					hipAngle  = 360*hipAngle/4096; 
					jointMsgProcess(hipAngle,HIP_MOTOR);
					can1SendMsg(jointSpeedMsg,5);				   		 	//�����Źؽ��ٶ�
					can1SendMsg(jointPositionMsg,5);				   	//�����ŹؽڽǶ�
				break;
				case 3:
					yawAngle  = getJointCoder(yawCoder);
					yawAngle  =  360*yawAngle/4096;
					jointMsgProcess(yawAngle,YAW_MOTOR);
					can1SendMsg(jointPositionMsg,5);				   		//����yaw�ؽڽǶ�
					can1SendMsg(jointSpeedMsg,5);				   		 	  //����yaw�ؽ��ٶ�
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
			ControlProcess();//��ɿ���	
		}
			
			
	} 
} 

