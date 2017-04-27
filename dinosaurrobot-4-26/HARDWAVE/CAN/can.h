#ifndef __CAN_H
#define __CAN_H
#include "sys.h"


#define PC_ID										0X00
#define POWER_BOARD_ID					0X01
#define LEFT_FRONT_BOARD_ID     0X02
#define LEFT_BACK_BOARD_ID      0X03
#define RIGHT_FRONT_BOARD_ID    0X04
#define RIGHT_BACK_BOARD_ID			0X05
#define HEADER_BOARD_ID      		0X06
#define TAIL_BOARD_ID					  0X07

#define FILTER_ID	RIGHT_FRONT_BOARD_ID

extern int can1RxFinishFlg,can2RxFinishFlg;
extern int8_t can1RxMsg[],can2RxMsg[],can2RxMsg1[];
extern int8_t can1TxMsg[];

u8 can1ModeInit(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
void can1SendMsg(int8_t* msg,u8 len);

u8 can1ReceiveMsg(u8 *buf);							//接收数据

u8 can2ModeInit(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
void can2SendMsg(uint8_t driver_id,uint8_t commamd,int32_t absolut_position,uint8_t length);						//发送数据

u8 can2ReceiveMsg(u8 *buf);							//接收数据

#endif

