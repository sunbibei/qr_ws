/**
  ******************************************************************************
  * @file    HARDWARE/MOTOR/motor.c 
  * @author  Liao
  * @version V1
  * @date    2017.01.05
  * @brief   MOTOR CONTROL.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "math.h"


int8_t jointPositionMsg[CAN1_PACKAGE_LENGTH],jointSpeedMsg[CAN1_PACKAGE_LENGTH];


int32_t  Get_neg(int32_t absolut_position)
{
	return absolut_position;
}
/*******************************************************************************
* Function Name  : can1MsgProcess
* Description    : This function handles can1 receive message.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void can1MsgProcess(int8_t *P_RxMsg)
{
	uint8_t i=0;			
	
	switch( P_RxMsg[i]&0x0f)
	{
			case KNEE_DRIVER_ID-1:
				Ctl_Msgs[0].frameType = P_RxMsg[i];		
				Ctl_Msgs[0].length =P_RxMsg[i+1];						
				Ctl_Msgs[0].jointangle = (int16_t)(256*((int16_t)(P_RxMsg[i+4])))+((int16_t)(P_RxMsg[i+3]) & 0x00ff);	//获取关节角度(弧度)		
				break;
				
			case HIP_DRIVER_ID-1:	
				Ctl_Msgs[1].frameType = P_RxMsg[i];		
				Ctl_Msgs[1].length =P_RxMsg[i+1];						
				Ctl_Msgs[1].jointangle = (int16_t)(256*((int16_t)(P_RxMsg[i+4])))+((int16_t)(P_RxMsg[i+3]) & 0x00ff);	//获取关节角度(弧度)		
				break;
			
			case YAW_DRIVER_ID-1:				
				Ctl_Msgs[2].frameType = P_RxMsg[i];		
				Ctl_Msgs[2].length =P_RxMsg[i+1];						
				Ctl_Msgs[2].jointangle = (int16_t)(256*((int16_t)(P_RxMsg[i+4])))+((int16_t)(P_RxMsg[i+3]) & 0x00ff);	//获取关节角度(弧度)		
				break;
			default:break;
	}
//		Ctl_Msgs.frameType = P_RxMsg[i];		
//		Ctl_Msgs.length =P_RxMsg[i+1];	
//		
//		jointTemp1 = (int16_t)(P_RxMsg[i+4]);
//		jointTemp2 = ((int16_t)(P_RxMsg[i+3]) & 0x00ff);	
//		
//		Ctl_Msgs.jointangle = (int16_t)(256*jointTemp1)+jointTemp2;	//获取关节角度(弧度)	
	
}

/*******************************************************************************
* Function Name  : can2MsgProcess
* Description    : This function handles can2 receive message.
* Input          : P_RxMsg
* Output         : None
* Return         : None
*******************************************************************************/
void can2MsgProcess(int8_t *P_RxMsg)
{
	uint8_t i=0,driverMode =0;
	uint8_t driverid = 0;						//驱动器id
	uint16_t jointangle = 0;					//关节角度
	uint32_t motorposition = 0;					//电机实际位置值
	
	driverid = P_RxMsg[1];						//获取当前驱动器ID
	driverMode = P_RxMsg[2];					//获取当前启动器指令模式
	motorposition = ((P_RxMsg[i+7]&0x000000ff)<<24)|((P_RxMsg[i+6]&0x000000ff)<<16)|((P_RxMsg[i+5]&0x000000ff)<<8)|(P_RxMsg[i+4]&0x000000ff);
//	jointangle = MOTORPOSITION_TO_ANGLE*motorposition;		//电机实际位置转换为弧度值
	
	switch(driverMode)
	{
		case MODE_GET_POSITION: can1TxMsg[i] = JOINT_ANGLE_SEND; break;
		case MODE_GET_SPEED:	can1TxMsg[i] = (JOINT_SPEED_SEND | FILTER_ID); break;
	}
	
	can1TxMsg[i+1] = 0X11;
	can1TxMsg[i+2] = 0X88;

	switch(driverid)
	{
		case KNEE_DRIVER_ID:
			motorposition = -motorposition;
			jointangle = NUM_CONSTANT*27.70885*motorposition/5160960+1.01*NUM_CONSTANT;
			can1TxMsg[i+3] = jointangle&0x00ff; 
			can1TxMsg[i+4] = (jointangle>>8)&0x00ff;
//			can1TxMsg[i+3] = can1TxMsg[i+3]+1;
//			can1TxMsg[i+4] = can1TxMsg[i+4]+1;
		break;
		case HIP_DRIVER_ID: 
			jointangle = NUM_CONSTANT*16.03155*motorposition/5160960+0.44*NUM_CONSTANT;
			can1TxMsg[i+5] = jointangle&0x00ff; 
			can1TxMsg[i+6] = (jointangle>>8)&0x00ff;
//			can1TxMsg[i+5] = can1TxMsg[i+5]+1; 
//			can1TxMsg[i+6] = can1TxMsg[i+6]+1;
		break;
		case YAW_DRIVER_ID: 
			jointangle = NUM_CONSTANT*16.03155*motorposition/5160960+0.44*NUM_CONSTANT;
			can1TxMsg[i+5] = jointangle&0x00ff; 
			can1TxMsg[i+6] = (jointangle>>8)&0x00ff;
//			can1TxMsg[i+5] = can1TxMsg[i+5]+1; 
//			can1TxMsg[i+6] = can1TxMsg[i+6]+1;
		break;
		default:break;
	
	}
	
}

void jointMsgProcess(u16 jointangle,char joint)
{
	char i = 0;

	jointPositionMsg[i] = JOINT_ANGLE_SEND | joint;
	jointSpeedMsg[i]    = JOINT_SPEED_SEND | joint;
	
	jointPositionMsg[i+1] = 0x11;
	jointSpeedMsg[i+1]    = 0x11;
	
	jointPositionMsg[i+2] = 0x88;
	jointSpeedMsg[i+2]    = 0x88;
	
	jointPositionMsg[i+3] = jointangle&0xff;
	jointSpeedMsg[i+3]    = jointangle&0xff;
	
	jointPositionMsg[i+4] = (jointangle>>8&0xff);
	jointSpeedMsg[i+4]    = (jointangle>>8&0xff);	
	
}
