 /**
  ******************************************************************************
  * @file    HARDWARE/MOTOR/adc.c 
  * @author  Liao
  * @version V1
  * @date    2017.03.31
  * @brief   joint angle coders data sample and process.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
 
 #include "main.h"
 #define N 200 //����ÿ�����ֵ������
 
/*******************************************************************************
* Function Name  : jointCoderInit
* Description    : This function is initialize the ADC of three joint angle coders.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/																   
void  jointCoderInit(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );	 	 //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   										 		//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA4 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;						//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  													//��λADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);									//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  
	ADC_Cmd(ADC1, ENABLE);						//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);					//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 				//����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 	//�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}				  

/*******************************************************************************
* Function Name  : Get_Adc
* Description    : This function is get the adcvalue of joint angle coders.
* Input          : adc channal:1)kneecoder(ADC_Channel_4);2)hipcoder(ADC_Channel_5);3)yawcoder(ADC_Channel_6)
* Output         : None
* Return         : ADC_GetConversionValue
*******************************************************************************/																   
u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_7Cycles5 );		//ADC1,ADCͨ��,����ʱ��Ϊ7.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);									//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));							//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);									//�������һ��ADC1�������ת�����
}


/*******************************************************************************
* Function Name  : getJointCoder
* Description    : The way of median average filter for joint angle coders.
* Input          : adc channal:1)kneecoder(ADC_Channel_4);2)hipcoder(ADC_Channel_5);3)yawcoder(ADC_Channel_6)
* Output         : None
* Return         : sum/(N-2)
*******************************************************************************/	
/**��λֵƽ���˲���**/

//u16 getJointCoder(u8 ch)
//{
//   u16 count,i,j,temp; 					//i��j��ð��������±������count�ǲ������ݶ�����±����
//   u16 value_buf[N]; 					// ����N������ֵ�Ĵ洢����
//   u32  sum = 0; 						//��ͱ��������ڴ洢����ֵ���ۼ�ֵ
//   for(count = 0;count < N;count ++)   //��������N������ֵ
//   {
//      value_buf[count] = Get_Adc(ch);
//   }
//   for (j=0;j<N-1;j++) 					//����������С����
//   {
//      for(i = 0;i< N-j;i++)
//      {
//         if ( value_buf[i]>value_buf[i+1] )
//         {
//            temp = value_buf[i];
//            value_buf[i] = value_buf[i+1];
//            value_buf[i+1] = temp;
//         }
//      }
//   }
//   for(count = 1;count < N-1;count++)
//    sum += value_buf[count]; 		//ȥ�����˵���С��������ֵ�����м��N-2������ֵ���
//   return (sum/(N-2));				// �����м�N-2������ֵ��ƽ��ֵ
//}

/*******************************************************************************
* Function Name  : getJointCoder
* Description    : The way of average filter for joint angle coders.
* Input          : adc channal:1)kneecoder(ADC_Channel_4);2)hipcoder(ADC_Channel_5);3)yawcoder(ADC_Channel_6)
* Output         : None
* Return         : sum/(N-2)
*******************************************************************************/
u16 getJointCoder(u8 ch)
{
    u32  sum=0; //��ͱ��������ڴ洢����ֵ���ۼ�ֵ
    char count;//�������ݶ�����±����
    for(count=0;count<N;count++) //��������N������ֵ�����ۼ�
    {
      sum+=Get_Adc(ch);
    }
    return (sum/N); //���ۼ�ֵ����ƽ��������Ϊ����ֵ
}

