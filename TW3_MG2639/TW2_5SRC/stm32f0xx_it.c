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

volatile unsigned char FlagFTgklj;//1:��ȷ��0.1m3��2:��ȷ��1m3
volatile unsigned char CntPulseAL;
volatile unsigned char FlagPulseAL;//PulseALʱ=0x55;PulseALʱ=0xAA
volatile unsigned char NumPulseAL;//PulseAL��������
volatile uint32_t CntFTgkss;//1��ˮ�ۻ���ʱ
volatile unsigned char CntPulseBL;
volatile unsigned char FlagPulseBL;//PulseALʱ=0x55;PulseALʱ=0xAA
extern unsigned char FlagLED;//
extern volatile uint16_t Cnt1ms;//ÿ��1ms

extern volatile unsigned char Cnt10ms;//��ʱ10���������
extern volatile uint16_t Cnt1000ms;//��ʱ1000���������

extern volatile unsigned char  CntRx2;//USART2���յ����ֽ��� 
extern volatile unsigned char  Rx2Start; //����״̬��־��0x55��ʼ���գ�0xAA�������
extern volatile unsigned char CntRec2Nms;//3.5�ֽڼ�����������3.5�ֽڱ�ʾһ֡���ݽ������
extern volatile unsigned char Cnt2DlyNms;//1֡������ɺ�����ʱ3.5�ֽڣ���ֹӰ������MODBUS�ڵ�
extern volatile unsigned char  ProUSART2; //Э����������־��0x55���������0xAA���������
extern volatile unsigned char  NumPro2;//USART2�ȴ��������ֽ���
extern volatile unsigned char USART2OrderMode; //�������ͣ�
extern volatile uint16_t USART2OrderWait; //��ʱ����������ģ�鷢��һ�������ó�3������޻ظ���ÿ��SysTick��1
//extern volatile unsigned char Usart2OrderTimeOut; //Usart1OrderWait����0ʱ��Usart1OrderTimeOut+1
extern unsigned char USART2OrderErr; //Usart1OrderTimeOut�ӵ�3ʱ,�ó�0x55��ʾһ��ͨѶ����,����Ϊ0xAA 

extern volatile uint16_t ADC1ConvData[3];
extern volatile uint16_t ADNdValue,ADBtyValue,ADTempValue;//����ת����ֵ
extern volatile uint32_t SumNdValue,SumBtyValue,SumTempValue;//64��ת����ֵ
extern volatile uint16_t AVNdValue,AVBtyValue,AVTempValue;//64��ת����ƽ��ֵ
extern uint8_t CntADSum;//adֵ�ۼӼ�����
extern char FTgkljAscii[];
extern char FTgkssAscii[];
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c�ж���,�����ۼ�����,�洢��EE484-487;����˲ʱ����,��EE�в��洢
extern uint8_t EEArray[];//��Ӧ488��ģ��EE��ַ

extern uint8_t FlagPwr;//Ϊ0x55->run,0xAA->poweroff
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;

extern const int8_t tbl_ascii[];//uart_GSM.c�ж���

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
	CntFTgkss++;//˲ʱ����������++
	
	//����Usart2�ж�һ֡�Ƿ����
	//if(FlagPwr==0x55){//����״̬
	if(1){//����״̬
		//1.����USART2����,���㳬ʱ
		if(Rx2Start == 0x55) CntRec2Nms ++;//10�ֽڼ�������CntRec2Nms>10ʱ��һ֡�������
		if(CntRec2Nms > 50)//ԭ��10������50
		{	
			Rx2Start = 0xAA;
			NumPro2 = CntRx2;
			Cnt2DlyNms ++;//1֡������ɺ�����ʱ10�ֽڣ���ֹӰ������MODBUS�ڵ�
			if(Cnt2DlyNms > 10) {ProUSART2 = 0x55;CntRec2Nms = 0;Cnt2DlyNms = 0;}	
		}
		//�ж�USART2�ȴ���Ӧ�Ƿ�ʱ
		if((USART2OrderMode!=0x00)&&(ProUSART2 == 0xAA)&&(USART2OrderErr==0xAA))//1.���ڵȴ���2.δ������ظ�֡��3.δ��ʱ
		{
			USART2OrderWait--;//��ʱ�������ݼ�
			if(USART2OrderWait==0) USART2OrderErr=0x55;//����ͨѶ��ʱ����
		}
		//2.��������
		if(PulseAL()){
			CntPulseAL++;
			if(CntPulseAL>50)
				CntPulseAL=50;
		}
		if(PulseAH()){
			if(CntPulseAL>40){//����40ms�ж�Ϊ1������,��������
				if(FlagFTgklj==1){//0.1m3һ������
					NumPulseAL++;
					EEArray[487]=NumPulseAL;
					if(NumPulseAL>=10){//10���� 1 ��
						NumPulseAL=0;
						FTgklj++;//�ۻ���������
						EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
						EEArray[486]=(FTgklj>>16) & 0xFF;
						//RefreshEE();//����Ƶ����дFLASH,��ֹ����10000��Щ��
						//FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
						FTgkljAscii[1]='0';FTgkljAscii[2]='0';//���ϴ��ۼ�������С��λ
						FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
						FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
						FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
						FTgkss=(uint32_t)(1/((double)CntFTgkss/3600000));//˲ʱ��������
						FTgkssAscii[1]=tbl_ascii[(uint8_t)((FTgkss>>28)&0x0F)];FTgkssAscii[2]=tbl_ascii[(uint8_t)((FTgkss>>24)&0x0F)];
						FTgkssAscii[3]=tbl_ascii[(uint8_t)((FTgkss>>20)&0x0F)];FTgkssAscii[4]=tbl_ascii[(uint8_t)((FTgkss>>16)&0x0F)];
						FTgkssAscii[5]=tbl_ascii[(uint8_t)((FTgkss>>12)&0x0F)];FTgkssAscii[6]=tbl_ascii[(uint8_t)((FTgkss>>8)&0x0F)];
						FTgkssAscii[7]=tbl_ascii[(uint8_t)((FTgkss>>4)&0x0F)];FTgkssAscii[8]=tbl_ascii[(uint8_t)(FTgkss&0x0F)];
						CntFTgkss=0;//���������㣬���¿�ʼ������ֱ����һ��ˮ
					}				
				}//if(FlagFTgklj==1)
				if(FlagFTgklj==2){//1m3һ������
						FTgklj++;//�ۻ���������
						EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
						EEArray[486]=(FTgklj>>16) & 0xFF;
						//RefreshEE();//����Ƶ����дFLASH,��ֹ����10000��Щ��
						//FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
						FTgkljAscii[1]='0';FTgkljAscii[2]='0';//���ϴ��ۼ�������С��λ
						FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
						FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
						FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
						FTgkss=(uint32_t)(1/((double)CntFTgkss/3600000));//˲ʱ��������
						FTgkssAscii[1]=tbl_ascii[(uint8_t)((FTgkss>>28)&0x0F)];FTgkssAscii[2]=tbl_ascii[(uint8_t)((FTgkss>>24)&0x0F)];
						FTgkssAscii[3]=tbl_ascii[(uint8_t)((FTgkss>>20)&0x0F)];FTgkssAscii[4]=tbl_ascii[(uint8_t)((FTgkss>>16)&0x0F)];
						FTgkssAscii[5]=tbl_ascii[(uint8_t)((FTgkss>>12)&0x0F)];FTgkssAscii[6]=tbl_ascii[(uint8_t)((FTgkss>>8)&0x0F)];
						FTgkssAscii[7]=tbl_ascii[(uint8_t)((FTgkss>>4)&0x0F)];FTgkssAscii[8]=tbl_ascii[(uint8_t)(FTgkss&0x0F)];
						CntFTgkss=0;//���������㣬���¿�ʼ������ֱ����һ��ˮ			
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
			if(CntPulseBL>150){//����150ms�ж�Ϊ1��,��������
				if(FTgklj==0) 
					FTgklj=0;
				else
					FTgklj--;
				EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
				EEArray[486]=(FTgklj>>16) & 0xFF;EEArray[487]=(FTgklj>>24) & 0xFF;
				//RefreshEE();//����Ƶ����дFLASH,��ֹ����10000��Щ��
				FTgkljAscii[1]=tbl_ascii[(uint8_t)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(uint8_t)((FTgklj>>24)&0x0F)];
				FTgkljAscii[3]=tbl_ascii[(uint8_t)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(uint8_t)((FTgklj>>16)&0x0F)];
				FTgkljAscii[5]=tbl_ascii[(uint8_t)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(uint8_t)((FTgklj>>8)&0x0F)];
				FTgkljAscii[7]=tbl_ascii[(uint8_t)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(uint8_t)(FTgklj&0x0F)];
			}
			CntPulseBL=0;
		}
		//����ADCֵ
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
