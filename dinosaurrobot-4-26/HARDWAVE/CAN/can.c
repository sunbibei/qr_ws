/**
  ******************************************************************************
  * @file    HARDWARE/CAN/can.c 
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

int8_t can1RxMsg[CAN1_PACKAGE_LENGTH],can2RxMsg[CAN2_PACKAGE_LENGTH2],can2RxMsg1[CAN2_PACKAGE_LENGTH2];
int8_t can1TxMsg[CAN1_PACKAGE_LENGTH];
u16 test = 0x02;
int can1RxFinishFlg = 0,can2RxFinishFlg = 0;

/*******************************************************************************
* Function Name  : can1ModeInit
* Description    : The initialization of can2.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 can1ModeInit(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	NVIC_InitTypeDef  		NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;			//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	      	//模式设置： mode:0,普通模式;1,回环模式; 
	//设置波特率
	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32位宽 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(FILTER_ID<<5);		//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;			//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//激活过滤器0

	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化
 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}  

/*******************************************************************************
* Function Name  : can2ModeInit
* Description    : The initialization of can2.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 can2ModeInit(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;

	NVIC_InitTypeDef  		NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PORTB时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;				//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM=DISABLE;				//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM=DISABLE;				//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;				//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 		//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;				//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	      		//模式设置： mode:0,普通模式;1,回环模式; 
	//设置波特率
	CAN_InitStructure.CAN_SJW=tsjw;					//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 				//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;					//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;    		//分频系数(Fdiv)为brp+1	
	CAN_Init(CAN2, &CAN_InitStructure);     		//初始化CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=14;	//过滤器14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 			//屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 			//32位宽 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(FILTER_ID<<5);				//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;					//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器14关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	return 0;
}

 
/*******************************************************************************
* Function Name  : CAN1_RX0_IRQHandler
* Description    : The receive interrupt service function of can1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/		    
void CAN1_RX0_IRQHandler(void)
{
	char i; 
	
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	if(RxMessage.StdId == FILTER_ID)
	{
		for(i = 0; i < CAN1_PACKAGE_LENGTH; i++)
		{
			can1RxMsg[i] = RxMessage.Data[i];
		}
		can1MsgProcess(can1RxMsg);			//处理接收数据
		
		can1RxFinishFlg = 1;

		
		
	}

}


/*******************************************************************************
* Function Name  : CAN2_RX0_IRQHandler
* Description    : The receive interrupt service function of can2.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
	int i;
  	CanRxMsg RxMessage;
    CAN_Receive(CAN2, 0, &RxMessage);
	for(i = 0; i < CAN2_PACKAGE_LENGTH2; i++)
	{
		can2RxMsg[i] = RxMessage.Data[i];
	}
	
//	if( can2RxMsg[0] == 0x08)
//	{	
//		
//		can2MsgProcess(can2RxMsg);

//	}
	
}

/*******************************************************************************
* Function Name  : can1SendMsg
* Description    : The send service function of can1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void can1SendMsg(int8_t* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId = FILTER_ID;		// 标准标识符 
//	TxMessage.ExtId=0x12;				// 设置扩展标示符 
	TxMessage.IDE = CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR = CAN_RTR_Data;		// 数据帧
	TxMessage.DLC = len;				// 要发送的数据长度
	for(i=0;i<len;i++)
	TxMessage.Data[i] = msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i = 0; 
	while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	//等待发送结束
 	for(i=0;i<len;i++)
	TxMessage.Data[i] = 0;
}

/*******************************************************************************
* Function Name  : can2SendMsg
* Description    : The send service function of can2.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void can2SendMsg(uint8_t driver_id,uint8_t commamd,int32_t absolut_position,uint8_t length)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
//	int32_t absolut_position = 0;

	
	TxMessage.StdId = driver_id;					// 标准标识符 
//	TxMessage.ExtId=0x02;							// 设置扩展标示符 
	TxMessage.IDE = CAN_Id_Standard; 				// 标准帧
	TxMessage.RTR = CAN_RTR_Data;					// 数据帧
	TxMessage.DLC = length;							// 要发送的数据长度
	
	TxMessage.Data[i] = length;
	TxMessage.Data[i+1] = driver_id;
	TxMessage.Data[i+2] = commamd;
	TxMessage.Data[i+3] = 0x00;
	
	if(length == 8)
	{
		TxMessage.Data[i+4] = absolut_position&0xff;
		TxMessage.Data[i+5] = (absolut_position>>8)&0xff;
		TxMessage.Data[i+6] = (absolut_position>>16)&0xff;
		TxMessage.Data[i+7] = (absolut_position>>24)&0xff;
	}	
	     
	mbox = CAN_Transmit(CAN2, &TxMessage);   
	i = 0; 
	while((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	//等待发送结束
	 
}

