/**
  ******************************************************************************
  * @file    stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    02-October-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

//* Includes ------------------------------------------------------------------
#include "stm32f0xx_it.h"
#include "main.h"

volatile unsigned char FlagFTgklj;//1:精确到0.1m3，2:精确到1m3
volatile unsigned char CntPulseAL;
volatile unsigned char FlagPulseAL;//PulseAL时=0x55;PulseAL时=0xAA
volatile unsigned char NumPulseAL;//PulseAL的脉冲数
volatile uint32_t CntFTgkss;//1方水累积用时
volatile unsigned char CntPulseBL;
volatile unsigned char FlagPulseBL;//PulseAL时=0x55;PulseAL时=0xAA
extern unsigned char FlagLED;//
extern volatile uint16_t Cnt1ms;//每次1ms

extern volatile unsigned char Cnt10ms;//延时10毫秒计数器
extern volatile uint16_t Cnt1000ms;//延时1000毫秒计数器

extern volatile unsigned char  CntRx2;//USART2接收到的字节数 
extern volatile unsigned char  Rx2Start; //接收状态标志，0x55开始接收，0xAA接收完成
extern volatile unsigned char CntRec2Nms;//3.5字节计数器，超过3.5字节表示一帧数据接收完成
extern volatile unsigned char Cnt2DlyNms;//1帧接收完成后再延时3.5字节，防止影响其他MODBUS节点
extern volatile unsigned char  ProUSART2; //协议解析允许标志，0x55允许解析，0xAA不允许解析
extern volatile unsigned char  NumPro2;//USART2等待解析的字节数
extern volatile unsigned char USART2OrderMode; //命令类型：
extern volatile uint16_t USART2OrderWait; //超时计数器，向模块发送一次命令置成3，如果无回复，每个SysTick减1
//extern volatile unsigned char Usart2OrderTimeOut; //Usart1OrderWait减到0时，Usart1OrderTimeOut+1
extern unsigned char USART2OrderErr; //Usart1OrderTimeOut加到3时,置成0x55表示一次通讯故障,正常为0xAA 

extern volatile uint16_t ADC1ConvData[3];
extern volatile uint16_t ADNdValue,ADBtyValue,ADTempValue;//单次转换的值
extern volatile uint32_t SumNdValue,SumBtyValue,SumTempValue;//64次转换的值
extern volatile uint16_t AVNdValue,AVBtyValue,AVTempValue;//64次转换的平均值
extern uint8_t CntADSum;//ad值累加计数器
extern char FTgkljAscii[];
extern char FTgkssAscii[];
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c中定义,工况累计流量,存储在EE484-487;工况瞬时流量,在EE中不存储
extern uint8_t EEArray[];//对应488个模拟EE地址

extern uint8_t FlagPwr;//为0x55->run,0xAA->poweroff
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;

extern const int8_t tbl_ascii[];//uart_GSM.c中定义

/** @addtogroup STM32F0308-Discovery_Demo
  * @{
  */

/** @addtogroup STM32F0XX_IT
  * @brief Interrupts driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	Cnt10ms++;
	Cnt1000ms++;
	// TimingDelay_Decrement(); 
	Cnt1ms++;
	CntFTgkss++;//瞬时流量计数器++
	
	//辅助Usart2判断一帧是否结束
	//if(FlagPwr==0x55){//运行状态
	if(1){//运行状态
		//1.辅助USART2接收,计算超时
		if(Rx2Start == 0x55) CntRec2Nms ++;//10字节计数器，CntRec2Nms>10时，一帧接收完毕
		if(CntRec2Nms > 50)//原来10，现在50
		{	
			Rx2Start = 0xAA;
			NumPro2 = CntRx2;
			Cnt2DlyNms ++;//1帧接收完成后再延时10字节，防止影响其他MODBUS节点
			if(Cnt2DlyNms > 10) {ProUSART2 = 0x55;CntRec2Nms = 0;Cnt2DlyNms = 0;}	
		}
		//判断USART2等待响应是否超时
		if((USART2OrderMode!=0x00)&&(ProUSART2 == 0xAA)&&(USART2OrderErr==0xAA))//1.正在等待，2.未接收完回复帧，3.未超时
		{
			USART2OrderWait--;//超时计数器递减
			if(USART2OrderWait==0) USART2OrderErr=0x55;//发生通讯超时故障
		}
		//2.计算流量
		if(PulseAL()){
			CntPulseAL++;
			if(CntPulseAL>50)
				CntPulseAL=50;
		}
		if(PulseAH()){
			if(CntPulseAL>40){//大于40ms判断为1次脉冲,更新流量
				if(FlagFTgklj==1){//0.1m3一个脉冲
					NumPulseAL++;
					EEArray[487]=NumPulseAL;
					if(NumPulseAL>=10){//10脉冲 1 方
						NumPulseAL=0;
						FTgklj++;//累积流量计算
						EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
						EEArray[486]=(FTgklj>>16) & 0xFF;
						//RefreshEE();//不能频繁改写FLASH,防止超过10000次些坏
						//FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
						FTgkljAscii[1]='0';FTgkljAscii[2]='0';//不上传累计流量的小数位
						FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
						FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
						FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
						FTgkss=(uint32_t)(1/((double)CntFTgkss/3600000));//瞬时流量计算
						FTgkssAscii[1]=tbl_ascii[(uint8_t)((FTgkss>>28)&0x0F)];FTgkssAscii[2]=tbl_ascii[(uint8_t)((FTgkss>>24)&0x0F)];
						FTgkssAscii[3]=tbl_ascii[(uint8_t)((FTgkss>>20)&0x0F)];FTgkssAscii[4]=tbl_ascii[(uint8_t)((FTgkss>>16)&0x0F)];
						FTgkssAscii[5]=tbl_ascii[(uint8_t)((FTgkss>>12)&0x0F)];FTgkssAscii[6]=tbl_ascii[(uint8_t)((FTgkss>>8)&0x0F)];
						FTgkssAscii[7]=tbl_ascii[(uint8_t)((FTgkss>>4)&0x0F)];FTgkssAscii[8]=tbl_ascii[(uint8_t)(FTgkss&0x0F)];
						CntFTgkss=0;//计数器清零，重新开始计数，直到下一方水
					}				
				}//if(FlagFTgklj==1)
				if(FlagFTgklj==2){//1m3一个脉冲
						FTgklj++;//累积流量计算
						EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
						EEArray[486]=(FTgklj>>16) & 0xFF;
						//RefreshEE();//不能频繁改写FLASH,防止超过10000次些坏
						//FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
						FTgkljAscii[1]='0';FTgkljAscii[2]='0';//不上传累计流量的小数位
						FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
						FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
						FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
						FTgkss=(uint32_t)(1/((double)CntFTgkss/3600000));//瞬时流量计算
						FTgkssAscii[1]=tbl_ascii[(uint8_t)((FTgkss>>28)&0x0F)];FTgkssAscii[2]=tbl_ascii[(uint8_t)((FTgkss>>24)&0x0F)];
						FTgkssAscii[3]=tbl_ascii[(uint8_t)((FTgkss>>20)&0x0F)];FTgkssAscii[4]=tbl_ascii[(uint8_t)((FTgkss>>16)&0x0F)];
						FTgkssAscii[5]=tbl_ascii[(uint8_t)((FTgkss>>12)&0x0F)];FTgkssAscii[6]=tbl_ascii[(uint8_t)((FTgkss>>8)&0x0F)];
						FTgkssAscii[7]=tbl_ascii[(uint8_t)((FTgkss>>4)&0x0F)];FTgkssAscii[8]=tbl_ascii[(uint8_t)(FTgkss&0x0F)];
						CntFTgkss=0;//计数器清零，重新开始计数，直到下一方水			
				}//if(FlagFTgklj==1)
			}//if(CntPulseAL>40){
			CntPulseAL=0;
		}
		if(PulseBL()){
			CntPulseBL++;
			if(CntPulseBL>200)
				CntPulseBL=200;
		}
		if(PulseBH()){
			if(CntPulseBL>150){//大于150ms判断为1方,更新流量
				if(FTgklj==0) 
					FTgklj=0;
				else
					FTgklj--;
				EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
				EEArray[486]=(FTgklj>>16) & 0xFF;EEArray[487]=(FTgklj>>24) & 0xFF;
				//RefreshEE();//不能频繁改写FLASH,防止超过10000次些坏
				FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
				FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
				FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
				FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
			}
			CntPulseBL=0;
		}
		//更新ADC值
	}//if(1)
}


 
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0x8.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
