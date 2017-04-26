#ifndef __MOTOR_H
#define __MOTOR_H	 
#include "sys.h"

#define abs(x) ((x)>0? (x):(-(x)))
#define small5(x) ((x)>5000? (5000):(x))
#define small1(x) ((x)<1000? (1000):(x))

#define NUM_CONSTANT 10000
#define MOTORPOSITION_TO_ANGLE		0.001

#define CAN1_PACKAGE_LENGTH 		0X08
#define CAN2_PACKAGE_LENGTH1    	0X04
#define CAN2_PACKAGE_LENGTH2 		0X08

#define JOINT_ANGLE_CTL             0X10
#define JOINT_SPEED_CTL				0X20
#define JOINT_ANGLE_SEND            0X40
#define JOINT_SPEED_SEND            0X60
//作用对象
#define KNEE_MOTOR           		0X01
#define HIP_MOTOR           		0X02
#define YAW_MOTOR           		0X03
#define POWER_BOARD					0X04
#define LEFT_FRONT_BOARD       		0X05
#define LEFT_BACK_BOARD       		0X06
#define RIGHT_FRONT_BOARD      		0X07
#define RIGHT_BACK_BOARD			0X08

#define KNEE_DRIVER_ID           	0X02
#define HIP_DRIVER_ID            	0X03
#define YAW_DRIVER_ID            	0X04
//驱动器工作模式
#define MODE_SET_SPEED				0X90
#define MODE_GET_SPEED				0X91
#define MODE_ABSOLUT_POSITION       0X99
#define MODE_GET_POSITION           0X9B


void can1MsgProcess(int8_t *P_RxMsg);
void can2MsgProcess(int8_t *P_RxMsg);
void motorControl(uint8_t MOTOR_ID,uint8_t commamd,uint16_t jointangle,uint8_t length);
void jointMsgProcess(u16 jointangle,char joint);

extern int8_t jointPositionMsg[CAN1_PACKAGE_LENGTH],jointSpeedMsg[CAN1_PACKAGE_LENGTH];

typedef struct structCanMsgPackage
{
    uint8_t   FrameType;     	// 设备类型
    uint8_t   Length;        	// 数据包类型，包含单帧/多帧，及数据帧的个数
	
	uint8_t   time; 			//反馈时间
	
	int16_t   jointangle;     	//关节角度

	int16_t   kneejointangle;     	//膝关节关节角度
	int16_t   hipjointangle;     	//髋关节角度
	int16_t   yawjointangle;//TODO     	//yaw关节角度
	
	int32_t   absolut_position;
	
	int16_t   knee_speed;
	int16_t   hip_speed;
	int16_t   yaw_speed;//TODO
}CanMsgPackageType;

#endif
