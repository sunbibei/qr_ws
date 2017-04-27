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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	      	//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(FILTER_ID<<5);		//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;			//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;				//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;				//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;				//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;				//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 		//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;				//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	      		//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw;					//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 				//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;					//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;    		//��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN2, &CAN_InitStructure);     		//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=14;	//������14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 			//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 			//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=(FILTER_ID<<5);				//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;					//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������14������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ1
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
		can1MsgProcess(can1RxMsg);			//�����������
		
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
	TxMessage.StdId = FILTER_ID;		// ��׼��ʶ�� 
//	TxMessage.ExtId=0x12;				// ������չ��ʾ�� 
	TxMessage.IDE = CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR = CAN_RTR_Data;		// ����֡
	TxMessage.DLC = len;				// Ҫ���͵����ݳ���
	for(i=0;i<len;i++)
	TxMessage.Data[i] = msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i = 0; 
	while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed)&&(i<0XFFF))
		i++;	//�ȴ����ͽ���
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

	
	TxMessage.StdId = driver_id;					// ��׼��ʶ�� 
//	TxMessage.ExtId=0x02;							// ������չ��ʾ�� 
	TxMessage.IDE = CAN_Id_Standard; 				// ��׼֡
	TxMessage.RTR = CAN_RTR_Data;					// ����֡
	TxMessage.DLC = length;							// Ҫ���͵����ݳ���
	
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
		i++;	//�ȴ����ͽ���
	 
}

