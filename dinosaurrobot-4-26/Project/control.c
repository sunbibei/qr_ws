/**
  ******************************************************************************
  * @file    Project/control.c 
  * @author  Wang
  * @version V1
  * @date    2017.01.05
  * @brief   PID CONTROL.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
	
#include "main.h"

struct Ctl_Msg_Struct Ctl_Msgs[Joint_Num];
struct PID_Struct PID[Joint_Num];
CanMsgPackageType canMsgPackage;

int16_t knee_pos_target = 0;
int16_t hip_pos_target = 0;
int16_t yaw_pos_target = 0;
int16_t knee_pos_current = 0;
int16_t hip_pos_current = 0;
int16_t yaw_pos_current = 0;

/*******************************************************************************
* Function Name  : PID_init
* Description    : using  formal parameter to initialize assigned PID controller
* Input          : P,I,D value and PID ID number
* Output         : None
* Return         : None
*******************************************************************************/
void PID_init(float P,float I,float D,int ID)
{
	PID[ID].Kp = P,PID[ID].Kd = D,PID[ID].Ki = I;	
}
/*******************************************************************************
* Function Name  : PID_control
* Description    : using PD control to calculate.
* Input          : input current_command and current_position as array
* Output         : None
* Return         : None
*******************************************************************************/
void PID_control(int current_command,int current_coder,int ID)
{	
		PID[ID].SetPos = current_command;
		PID[ID].ActualPos = current_coder;
		PID[ID].err = PID[ID].SetPos - PID[ID].ActualPos;
		PID[ID].result = PID[ID].Kp *PID[ID].err + PID[ID].Kd * (PID[ID].err - PID[ID].last_err);
	  //PID[ID].result = PID[ID].Kp *PID[ID].err ;
		PID[ID].last_err = PID[ID].err;
}


void ControlProcess()
{	
	uint8_t i=0;	
	int32_t absolut_position = 0;
	
	int16_t current_position = 0;
	int16_t motor_hipAngle = 0;
	int16_t knee_motor_speed = 0;
	int16_t hip_motor_speed = 0;
	int16_t yaw_motor_speed = 0;
	int j=0;
	for(j=0;j<Joint_Num;j++)
	{
		int cammandType = Ctl_Msgs[j].frameType&0xf0;			//获取主索引值，即帧具体功能
		int targetobject = Ctl_Msgs[j].frameType&0x0f;		//获取副索引值，即作用对象
		
		canMsgPackage.FrameType =   Ctl_Msgs[j].frameType;
		canMsgPackage.Length =  Ctl_Msgs[j].length;
		
		PID_init(1,0,0,0);//float P,float I,float D,int16_t ID,yaw 0
		PID_init(2.5,0,5,1);//float P,float I,float D,int16_t ID,hip 1
		PID_init(1,0,0,2);//float P,float I,float D,int16_t ID,knee 2
	
		switch(targetobject)
		{
			case KNEE_DRIVER_ID-1:
				canMsgPackage.kneejointangle =  Ctl_Msgs[j].jointangle;
			  if(can1RxFinishFlg == 1)
				{
					knee_pos_target = 19197 - canMsgPackage.kneejointangle;
//					can1RxFinishFlg = 0;
				}
				knee_pos_current = (kneeAngle-Knee_Minangle)*NUM_CONSTANT/57.3;
				PID_control(knee_pos_target,knee_pos_current , 0);//TODO
			
				knee_motor_speed = PID[0].result;
									
			  can2SendMsg(KNEE_DRIVER_ID,MODE_SET_SPEED,knee_motor_speed,8);			
				
				break;
				
			case HIP_DRIVER_ID-1:
				canMsgPackage.hipjointangle =  Ctl_Msgs[j].jointangle;		
				if(can1RxFinishFlg == 1)
				{
					hip_pos_target = 12566 - canMsgPackage.hipjointangle;
					//can1RxFinishFlg = 0;
				}
				hip_pos_current = (hipAngle-Hip_Minangle)*NUM_CONSTANT/57.3;
			
				PID_control(hip_pos_target,hip_pos_current , 1);//TODO
			
				hip_motor_speed = PID[1].result;
			
				can2SendMsg(HIP_DRIVER_ID,MODE_SET_SPEED,hip_motor_speed,8); 
			
			break;
			
//			case YAW_DRIVER_ID-1://TODO		
//				canMsgPackage.yawjointangle =  Ctl_Msgs[j].jointangle;		
//				if(can1RxFinishFlg == 1)
//				{
//					yaw_pos_target = 8202 - canMsgPackage.yawjointangle;
//				}
//				yaw_pos_current = (Yaw_Maxangle-yawAngle)*NUM_CONSTANT/57.3;
//			
//				PID_control(yaw_pos_target,yaw_pos_current , 2);//TODO
//			
//				yaw_motor_speed = PID[2].result;
//			
//				can2SendMsg(YAW_DRIVER_ID,MODE_SET_SPEED,yaw_motor_speed,8);	
//							
//			break;
			default:break;
		}
	}
	can1RxFinishFlg = 0;
	
}

