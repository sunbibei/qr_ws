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
 #define N 200 //设置每组采样值的数量
 
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );	 	 //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   										 		//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA4 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;						//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  													//复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);									//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);						//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);					//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 				//开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 	//等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

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
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_7Cycles5 );		//ADC1,ADC通道,采样时间为7.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);									//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));							//等待转换结束

	return ADC_GetConversionValue(ADC1);									//返回最近一次ADC1规则组的转换结果
}


/*******************************************************************************
* Function Name  : getJointCoder
* Description    : The way of median average filter for joint angle coders.
* Input          : adc channal:1)kneecoder(ADC_Channel_4);2)hipcoder(ADC_Channel_5);3)yawcoder(ADC_Channel_6)
* Output         : None
* Return         : sum/(N-2)
*******************************************************************************/	
/**中位值平均滤波法**/

//u16 getJointCoder(u8 ch)
//{
//   u16 count,i,j,temp; 					//i，j是冒泡排序的下标变量，count是采样数据读入的下标变量
//   u16 value_buf[N]; 					// 缓冲N个采样值的存储变量
//   u32  sum = 0; 						//求和变量，用于存储采样值的累加值
//   for(count = 0;count < N;count ++)   //连续读入N个采样值
//   {
//      value_buf[count] = Get_Adc(ch);
//   }
//   for (j=0;j<N-1;j++) 					//气泡排序，由小到大
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
//    sum += value_buf[count]; 		//去掉两端的最小和最大采样值，对中间的N-2个采样值求和
//   return (sum/(N-2));				// 返回中间N-2个采样值的平均值
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
    u32  sum=0; //求和变量，用于存储采样值的累加值
    char count;//采样数据读入的下标变量
    for(count=0;count<N;count++) //连续读入N个采样值，并累加
    {
      sum+=Get_Adc(ch);
    }
    return (sum/N); //讲累加值进行平均计算作为返回值
}

