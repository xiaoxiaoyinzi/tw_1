/*****************************************************************************
  *�ļ���: F051test/Init.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/06
  *��@@������ʼ��
********************************************************************************/
#include "main.h"

uint8_t FlagRCC;////0xAA->HSI,//0x55->HSE

extern uint8_t FlagPwr;//Ϊ0x55->run,0xAA->poweroff
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;
extern uint8_t EEArray[];//��Ӧ488��ģ��EE��ַ

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0;
//����FlashEE����
uint8_t FlagFirstRun=0;

char APN[20];//EE��400~419��ASCII����ʽ
char USER[12];//EE��420~431��ASCII����ʽ
char PWD[8];//EE��432~439��ASCII����ʽ
char IPAddr[16];//EE��440~455��ASCII����ʽ
char PortID[4];//EE��456~459��ASCII����ʽ
char CircleAD[2];//EE��460~461��ASCII����ʽ
char CircleGPRS[2];//EE��464~465��ASCII����ʽ
char DeviceID[12];//EE��468~479�豸ʶ���,��11�ֽ���Ч��12=4*3��DeviceID[11]��дFlashʱ���
extern volatile uint32_t FTgklj,FTgkss;//�����ۼ�����,�洢��EE484-487;����˲ʱ����,��EE�в��洢
extern volatile unsigned char FlagFTgklj;//1:��ȷ��0.1m3��2:��ȷ��1m3
extern volatile unsigned char NumPulseAL;//PulseAL��������

extern char TimeAscii[11];//rtc.c����,ʱ���ASCII�룬WYYMMDDHHMM
//extern uint8_t Week,Year,Month,Date,Hours,Minutes,Seconds;//rtc.c����,BCD���ʽ������ʱ��
//extern const int8_t tbl_ascii[];//uart_GSM.c�ж���
//uint16_t Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
const char DeviceIDDefault[12]={'0','5','3','1','0','0','0','0','0','0','4','\0'};//05310000001
const char APNDefault[20]={'c','m','n','e','t','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};//cmnet
const char USERDefault[12]={'U','S','E','R','\0','\0','\0','\0','\0','\0','\0','\0'};//USER
const char PWDDefault[8]={'P','W','D','\0','\0','\0','\0','\0'};//USER
//const char IPAddrDefault[16]={'5','8','.','5','6','.','2','4','7','.','6','6','\0','\0','\0','\0'};//58.56.247.66
const char IPAddrDefault[16]={'1','2','4','.','1','2','8','.','2','0','0','.','3','0','\0','\0'};//124.128.200.30
const char PortIDDefault[4]={'5','2','2','1'};//5007
const char CircleADDefault[2]={'0','5'};//0x05,��λ����
const char CircleGPRSDefault[2]={'0','1'};//0x01����λСʱ

extern char FTgkljAscii[];//ADC_Calcu.c�ж���
extern const int8_t tbl_ascii[];//uart_GSM.c�ж���

void Reinit_RCC_HSI(void)//�ض���ϵͳʼʱ��ΪHSI
{	
	FlagRCC=0xAA;//0xAA->HSI
	RCC_HSEConfig(RCC_HSE_OFF);//RCC_HSE_OFF/RCC_HSE_ON/RCC_HSE_Bypass(��OSC_IN��������)
	//RCC_WaitForHSEStartUp();//����SUCCESS��ERROR
	
	RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSIʱ��У׼��ֵ���������
	RCC_HSICmd(ENABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14ʱ��У׼��ֵ���������
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14ΪADCר������
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	//RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(��OSC32_IN��������)
	//RCC_LSEDriveConfig(RCC_LSEDrive_MediumLow);//����ΪRCC_LSEDrive_Low��RCC_LSEDrive_MediumLow��RCC_LSEDrive_MediumHigh��RCC_LSEDrive_High
	//����ѡ���ĸ������������ⲿ32.768kHz��ʱ�ӣ���������ǿ�ȣ�����ϵͳ³���Ժ͵͹���Ҫ������ѡ��
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4);//����1ȡֵRCC_PLLSource_HSI_Div2��RCC_PLLSource_PREDIV1��HSE��Ԥ��Ƶ��
	//����2ΪPLL��Ƶ��RCC_PLLMul_2��RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,����PLLԤ��Ƶ��������PLL��ֹʱ���
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE
	

	//RCC_ClockSecuritySystemCmd(DISABLE);//ENABLE or DISABLE,����ʱ�Ӱ�ȫ���ϵͳ
	//RCC_MCOConfig(RCC_MCOSource_LSI);//����MCOʱ������ŵ�ʱ��Դ,RCC_MCOSource_**,
	//**ΪNoClock/HSI14/LSI/LSE/HSI/HSE/SYSCLK/PLLCLK_Div2
	//RCC_MCOConfig(RCC_MCOSource_LSI,RCC_MCOPrescaler_2);//����2ȡֵΪRCC_MCOPrescaler_**,**Ϊ(1/2/4/8/16/32/64/128)
	
	//����ϵͳʱ��ΪPLL
	//while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//�ȴ�PLL�ȶ�
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	//while(RCC_GetSYSCLKSource()!=0x08);//0x00:HSI, 0x04:HSE, 0x08:PLL
	//����ϵͳʱ��ΪHSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x00);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	RCC_HCLKConfig(RCC_SYSCLK_Div16);//AHB����(core,DMA)ʱ��,RCC_SYSCLK_Div*,*Ϊ1,2,4,8,16,64,128,256,512
	RCC_PCLKConfig(RCC_HCLK_Div1);//APB����(����)ʱ��,RCC_HCLK_Div*,*Ϊ1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_LSE);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,������������32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	//RCC_RTCCLKCmd(DISABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,���ݼĴ���,
	
	//RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
	//����1��RCC_AHBPeriph_GPIOA~F��RCC_AHBPeriph_TS��RCC_AHBPeriph_CRC,RCC_AHBPeriph_FLITF,RCC_AHBPeriph_SRAM,RCC_AHBPeriph_DMA1
	
	//RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//����1��RCC_APB2Periph_SYSCFG,RCC_APB2Periph_ADC1,RCC_APB2Periph_TIM1,RCC_APB2Periph_SPI1,
	//RCC_APB2Periph_USART1,RCC_APB2Periph_TIM15~7,RCC_APB2Periph_DBGMCU
	
	//RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//����1��RCC_APB1Periph_TIM2/3/6/14,RCC_APB1Periph_WWDG,RCC_APB1Periph_SPI2,RCC_APB1Periph_USART2,
	//RCC_APB1Periph_I2C1~2,RCC_APB1Periph_PWR,RCC_APB1Periph_ADC,RCC_APB1Periph_CEC
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//�ж�ʹ��,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,
	//RCC_IT_PLLRDY,RCC_IT_HSI14RDY,
	//FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);//stm32f0xx_rcc.c��1473��
	//RCC_ClearFlag();//���reset flags
	//ITStatus RCC_GetITStatus(uint8_t RCC_IT);//stm32f0xx_rcc.c��1544��
	//RCC_ClearITPendingBit(uint8_t RCC_IT);//stm32f0xx_rcc.c��1577��
	
}
void Reinit_RCC_HSE(void)//�ض���ϵͳʼʱ��HSE
{	
	FlagRCC=0x55;//0x55->HSE
	RCC_HSEConfig(RCC_HSE_ON);//RCC_HSE_OFF/RCC_HSE_ON/RCC_HSE_Bypass(��OSC_IN��������)
	RCC_WaitForHSEStartUp();//����SUCCESS��ERROR
	
	//RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSIʱ��У׼��ֵ���������
	//RCC_HSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14ʱ��У׼��ֵ���������
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14ΪADCר������
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	//RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(��OSC32_IN��������)
	//RCC_LSEDriveConfig(RCC_LSEDrive_High);//����ΪRCC_LSEDrive_Low��RCC_LSEDrive_MediumLow��RCC_LSEDrive_MediumHigh��RCC_LSEDrive_High
	//����ѡ���ĸ������������ⲿ32.768kHz��ʱ�ӣ���������ǿ�ȣ�����ϵͳ³���Ժ͵͹���Ҫ������ѡ��
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8);//����1ȡֵRCC_PLLSource_HSI_Div2��RCC_PLLSource_PREDIV1��HSE��Ԥ��Ƶ��
	//����2ΪPLL��Ƶ��RCC_PLLMul_2��RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,����PLLԤ��Ƶ��������PLL��ֹʱ���
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE
	

	//RCC_ClockSecuritySystemCmd(DISABLE);//ENABLE or DISABLE,����ʱ�Ӱ�ȫ���ϵͳ
	//RCC_MCOConfig(RCC_MCOSource_LSI);//����MCOʱ������ŵ�ʱ��Դ,RCC_MCOSource_**,
	//**ΪNoClock/HSI14/LSI/LSE/HSI/HSE/SYSCLK/PLLCLK_Div2
	//RCC_MCOConfig(RCC_MCOSource_LSI,RCC_MCOPrescaler_2);//����2ȡֵΪRCC_MCOPrescaler_**,**Ϊ(1/2/4/8/16/32/64/128)
	
	//����ϵͳʱ��ΪPLL
	//while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//�ȴ�PLL�ȶ�
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	//while(RCC_GetSYSCLKSource()!=0x08);//0x00:HSI, 0x04:HSE, 0x08:PLL
	//����ϵͳʱ��ΪHSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x04);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB����(core,DMA)ʱ��,RCC_SYSCLK_Div*,*Ϊ1,2,4,8,16,64,128,256,512
	RCC_PCLKConfig(RCC_HCLK_Div1);//APB����(����)ʱ��,RCC_HCLK_Div*,*Ϊ1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_LSE);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,������������32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	//RCC_RTCCLKCmd(ENABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,���ݼĴ���,
	
	//RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
	//����1��RCC_AHBPeriph_GPIOA~F��RCC_AHBPeriph_TS��RCC_AHBPeriph_CRC,RCC_AHBPeriph_FLITF,RCC_AHBPeriph_SRAM,RCC_AHBPeriph_DMA1
	
	//RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//����1��RCC_APB2Periph_SYSCFG,RCC_APB2Periph_ADC1,RCC_APB2Periph_TIM1,RCC_APB2Periph_SPI1,
	//RCC_APB2Periph_USART1,RCC_APB2Periph_TIM15~7,RCC_APB2Periph_DBGMCU
	
	//RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//����Ӧ�������ʼ�����޸�,
	//����1��RCC_APB1Periph_TIM2/3/6/14,RCC_APB1Periph_WWDG,RCC_APB1Periph_SPI2,RCC_APB1Periph_USART2,
	//RCC_APB1Periph_I2C1~2,RCC_APB1Periph_PWR,RCC_APB1Periph_ADC,RCC_APB1Periph_CEC
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//�ж�ʹ��,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,
	//RCC_IT_PLLRDY,RCC_IT_HSI14RDY,
	//FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);//stm32f0xx_rcc.c��1473��
	//RCC_ClearFlag();//���reset flags
	//ITStatus RCC_GetITStatus(uint8_t RCC_IT);//stm32f0xx_rcc.c��1544��
	//RCC_ClearITPendingBit(uint8_t RCC_IT);//stm32f0xx_rcc.c��1577��
	
}
void Systick_Init(void)
{
	if(FlagRCC==0xAA) 
		SysTick_Config(500);//ϵͳʱ��ΪHSI(8MHz)��16��Ƶ,1000��SYSCLK���ڸպ���1ms
	else if(FlagRCC==0x55)
		SysTick_Config(4000);//ϵͳʱ��ΪHSE(4MHz)��4��Ƶ,1000��SYSCLK���ڸպ���1ms
//	SysTick_Config(40000);//ϵͳʱ��ΪPLL:40MHz,40000��SYSCLK���ڸպ���1ms
}

void Init_GPIO(void) 
{
	//*
	GPIO_InitTypeDef GPIO_InitStruct; //����ṹ��
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);//
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, DISABLE);//
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);//
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);//
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, DISABLE);//
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOF);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4;//LED:RUN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	VCCTempOff();//�ر�LMT84
	LEDSetOff();
	LEDRunOff();
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11;//pulseA,pulseB
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	
	//�������ս�
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_15;//��
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;//EN5V\BTY_EN\EN4V\GPRS_ON
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_13);//EN4V,�͵�ƽ��Ч
	GPIO_SetBits(GPIOB, GPIO_Pin_14);//GPRSON,�͵�ƽ��Ч
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//TX2\RX2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;//PA8���RS485����ģʽ��Ӧ����Init_GPIO()��,ѭ�����
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_15;//��
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	
	//SWD�ڣ���ʽ����ʱ���ã�����ʱ����
	/*
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;//��
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_13);
	GPIO_ResetBits(GPIOA, GPIO_Pin_14);
	*/
	
	//��ʼ��RS485�������ó���Σ����Ժú�ر�
	//USART1Init();
	//USART1VarInit();
	CloseUSART1();
}
void ReadFirstRun(void)//��ȡ�������б�־,����ǵ�һ�����У�д��Ĭ��ֵ���޸ĳ������б�־
{
	uint32_t EEData;
	uint8_t i;
	//uint8_t FlagFirstRun=0;
	EEData=FlashEERead(480);
	FlagFirstRun=(uint8_t)(EEData & 0x000000FF);//480
	EEArray[480]=FlagFirstRun;
	FlagFTgklj=(uint8_t)((EEData>>8) & 0x000000FF);//481
	if(FlagFirstRun!=1){//��һ�����У���Ĭ��ֵд��EE,"05310000001'\0'"
		for(i=0;i<12;i+=4){//DeviceIDDefault
			EEData=(((uint32_t)DeviceIDDefault[i+3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)DeviceIDDefault[i+2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)DeviceIDDefault[i+1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)DeviceIDDefault[i]) & 0x000000FF;
			FlashEEWrite(468+i,EEData);
		}
		for(i=0;i<20;i+=4){//APNDefault,"cmnet'\0'"
			EEData=(((uint32_t)APNDefault[i+3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)APNDefault[i+2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)APNDefault[i+1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)APNDefault[i]) & 0x000000FF;
			FlashEEWrite(400+i,EEData);
		}
		for(i=0;i<12;i+=4){//USERDefault,"USER"
			EEData=(((uint32_t)USERDefault[i+3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)USERDefault[i+2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)USERDefault[i+1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)USERDefault[i]) & 0x000000FF;
			FlashEEWrite(420+i,EEData);
		}
		for(i=0;i<8;i+=4){//PWDDefault,"PWD"
			EEData=(((uint32_t)PWDDefault[i+3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)PWDDefault[i+2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)PWDDefault[i+1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)PWDDefault[i]) & 0x000000FF;
			FlashEEWrite(432+i,EEData);
		}
		for(i=0;i<16;i+=4){//IPAddrDefault,"124.128.200.30"
			EEData=(((uint32_t)IPAddrDefault[i+3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)IPAddrDefault[i+2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)IPAddrDefault[i+1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)IPAddrDefault[i]) & 0x000000FF;
			FlashEEWrite(440+i,EEData);
		}
		//PortIDDefault,"5007"
			EEData=(((uint32_t)PortIDDefault[3]) << 24) & 0xFF000000;
			EEData |= (((uint32_t)PortIDDefault[2]) << 16) & 0x00FF0000;
			EEData |= (((uint32_t)PortIDDefault[1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)PortIDDefault[0]) & 0x000000FF;
			FlashEEWrite(456,EEData);
		//CircleADDefault,"05"
			EEData = (((uint32_t)CircleADDefault[1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)CircleADDefault[0]) & 0x000000FF;
			FlashEEWrite(460,EEData);
		//CircleGPRSDefault,"01"
			EEData = (((uint32_t)CircleGPRSDefault[1]) << 8) & 0x0000FF00;
			EEData |= ((uint32_t)CircleGPRSDefault[0]) & 0x000000FF;
			FlashEEWrite(464,EEData);
		//�޸ĳ������б�־
		FlashEEWrite(480,0x00000101);//�����й���0.1m3/h
		EEArray[480]=0x01;//��ֹ�޸�ʱ����0xFFд��EE��480λ��
		EEArray[481]=0x01;//д��FlagFTgklj
		FlashEEWrite(484,0x00000000);//��һ�����У��ۻ���Ϊ0 
		FTgklj=0;
		EEArray[484]=0x00;EEArray[485]=0x00;EEArray[486]=0x00;EEArray[487]=0x00;//��0д��EE��484λ��
	}else{
		__nop();
	}
}
void ReadConfig(void)//��ȡ�洢��FlashEE�е����ò���
{
	uint8_t Index;
	uint32_t EEdata;
	//char DeviceID[12];//EE��400~419��ASCII����ʽ
	//char APN[20];//EE��400~419��ASCII����ʽ
	//char USER[12];//EE��420~431��ASCII����ʽ
	//char PWD[8];//EE��432~439��ASCII����ʽ
	//char IPAddr[16];//EE��440~455��ASCII����ʽ
	//char PortID[4];//EE��456~459��ASCII����ʽ
	//char CircleAD[2];//EE��460~461��ASCII����ʽ
	//char CircleGPRS[2];//EE��464~465��ASCII����ʽ
	for(Index=0;Index<20;Index+=4){
		EEdata=FlashEERead(400+Index);
		APN[Index]=(char)(EEdata & 0x000000FF);
		APN[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		APN[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		APN[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	APN[19]='\0';//ĩ�ֽ����'\0',�����ϵ�ʱ������δ�������ã���ֹGPRS����ʱ����
	for(Index=0;Index<20;Index++)
		EEArray[400+Index]=APN[Index];
	
	for(Index=0;Index<12;Index+=4){
		EEdata=FlashEERead(420+Index);
		USER[Index]=(char)(EEdata & 0x000000FF);
		USER[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		USER[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		USER[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	USER[11]='\0';//ĩ�ֽ����'\0',�����ϵ�ʱ������δ�������ã���ֹGPRS����ʱ����
	for(Index=0;Index<12;Index++)
		EEArray[420+Index]=USER[Index];
	
	for(Index=0;Index<8;Index+=4){
		EEdata=FlashEERead(432+Index);
		PWD[Index]=(char)(EEdata & 0x000000FF);
		PWD[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		PWD[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		PWD[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	PWD[7]='\0';//ĩ�ֽ����'\0',�����ϵ�ʱ������δ�������ã���ֹGPRS����ʱ����
	for(Index=0;Index<8;Index++)
		EEArray[432+Index]=PWD[Index];
	
	for(Index=0;Index<16;Index+=4){
		EEdata=FlashEERead(440+Index);
		IPAddr[Index]=(char)(EEdata & 0x000000FF);
		IPAddr[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		IPAddr[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		IPAddr[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	IPAddr[15]='\0';//ĩ�ֽ����'\0',�����ϵ�ʱ������δ�������ã���ֹGPRS����ʱ����
	for(Index=0;Index<16;Index++)
		EEArray[440+Index]=IPAddr[Index];
	
	EEdata=FlashEERead(456);
	PortID[0]=(char)(EEdata & 0x000000FF);
	PortID[1]=(char)((EEdata>>8) & 0x000000FF);
	PortID[2]=(char)((EEdata>>16) & 0x000000FF);
	PortID[3]=(char)((EEdata>>24) & 0x000000FF);
	for(Index=0;Index<4;Index++)
		EEArray[456+Index]=PortID[Index];
	
	EEdata=FlashEERead(460);
	CircleAD[0]=(char)(EEdata & 0x000000FF);
	CircleAD[1]=(char)((EEdata>>8) & 0x000000FF);
	EEArray[460]=CircleAD[0];
	EEArray[461]=CircleAD[1];
	
	EEdata=FlashEERead(464);
	CircleGPRS[0]=(char)(EEdata & 0x000000FF);
	CircleGPRS[1]=(char)((EEdata>>8) & 0x000000FF);	
	EEArray[464]=CircleGPRS[0];
	EEArray[465]=CircleGPRS[1];
	for(Index=0;Index<12;Index+=4){
		EEdata=FlashEERead(468+Index);
		DeviceID[Index]=(char)(EEdata & 0x000000FF);
		DeviceID[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		DeviceID[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		DeviceID[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	DeviceID[11]='\0';//ĩ�ֽ����'\0',�����ϵ�ʱ������δ�������ã���ֹGPRS����ʱ����
	for(Index=0;Index<12;Index++){
		EEArray[468+Index]=DeviceID[Index];
	}
	EEdata=FlashEERead(480);
	EEArray[480]=(char)(EEdata & 0x000000FF);
	FlagFTgklj=(char)((EEdata>>8) & 0x000000FF);
	EEArray[481]=FlagFTgklj;
	EEdata=FlashEERead(484);
	FTgklj=EEdata & 0x00FFFFFF;//�ۼ���������ռ 3 �ֽ�
	NumPulseAL = (EEdata>>24) & 0xFF;////�ۼ���������ռ 1 �ֽ�
	EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
	EEArray[486]=(FTgklj>>16) & 0xFF;EEArray[487]=NumPulseAL;

	//FTgkljAscii[1]=tbl_ascii[(char)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(char)((FTgklj>>24)&0x0F)];
	FTgkljAscii[1]='0';FTgkljAscii[2]='0';
	FTgkljAscii[3]=tbl_ascii[(char)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(char)((FTgklj>>16)&0x0F)];
	FTgkljAscii[5]=tbl_ascii[(char)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(char)((FTgklj>>8)&0x0F)];
	FTgkljAscii[7]=tbl_ascii[(char)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(char)(FTgklj&0x0F)];
	
	ReadBiaoding();//��EE�б궨����
}

void InitRun(void)
{
	Reinit_RCC_HSI();
	//Reinit_RCC_HSE();
	Systick_Init();
	Init_GPIO();
	//Init_ADC();
	//Init_ADCDMA();
	
	//Usart1VarInit();
	//Init_USART1();
	//USART2VarInit();
	//Init_USART2();

	//InitFlashEE();//���4ҳFlashEE
	ReadConfig();//��ȡ�洢��FlashEE�е����ò���//
	ReadFirstRun();
	ReadConfig();//��ȡ�洢��FlashEE�е����ò���//

	RTCConfig();//������ʼ��RTC
	GetTimeAscii();

}

void InitPwrOff(void)//�ػ�������͹���ģʽ��ʼ��
{
	//GPIO_InitTypeDef GPIO_InitStruct; //����ṹ��
	/*
	RCC_HSEConfig(RCC_HSE_OFF);
	RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSIʱ��У׼��ֵ���������
	RCC_HSICmd(ENABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14ʱ��У׼��ֵ���������
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14ΪADCר������
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(��OSC32_IN��������)
	RCC_LSEDriveConfig(RCC_LSEDrive_Low);//����ΪRCC_LSEDrive_Low��RCC_LSEDrive_MediumLow��RCC_LSEDrive_MediumHigh��RCC_LSEDrive_High
	//����ѡ���ĸ������������ⲿ32.768kHz��ʱ�ӣ���������ǿ�ȣ�����ϵͳ³���Ժ͵͹���Ҫ������ѡ��
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_10);//����1ȡֵRCC_PLLSource_HSI_Div2��RCC_PLLSource_PREDIV1��HSE��Ԥ��Ƶ��
	//����2ΪPLL��Ƶ��RCC_PLLMul_2��RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,����PLLԤ��Ƶ��������PLL��ֹʱ���
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE

	//����ϵͳʱ��ΪHSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x00);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	//RCC_HCLKConfig(RCC_SYSCLK_Div1);//15625Hz.AHB����(core,DMA)ʱ��,RCC_SYSCLK_Div*,*Ϊ1,2,4,8,16,64,128,256,512
	FLASH_PrefetchBufferCmd(ENABLE);//RCC_SYSCLK_Div����1������ʹ��FlashԤ���Ĵ���
	RCC_HCLKConfig(RCC_SYSCLK_Div64);//15625Hz.AHB����(core,DMA)ʱ��,RCC_SYSCLK_Div*,*Ϊ1,2,4,8,16,64,128,256,512
	//RCC_PCLKConfig(RCC_HCLK_Div1);//APB����(����)ʱ��,RCC_HCLK_Div*,*Ϊ1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_HSI_Div244);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,������������32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	RCC_RTCCLKCmd(DISABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,���ݼĴ���,
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//�ж�ʹ��,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,

	SysTick_Config(15);//ϵͳʱ��ΪHSI:8MHz,8000��SYSCLK���ڸպ���1ms
	//SysTick_Config(8000);//ϵͳʱ��ΪHSI:8MHz,8000��SYSCLK���ڸպ���1ms
	*/
	//��GPIOA\GPIOB��ر���������
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);//ADCʱ��ʹ��
	ADC_Cmd(ADC1, DISABLE); //�ر�ADC1   
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);//DMAʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE ); //usart1���߹ر�
	USART_Cmd(USART1, DISABLE);//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE );
	USART_Cmd(USART2, DISABLE);
	/*
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;//PWR
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//LEDB
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	*/
}
/************************ (C) ��Ȩ �ۿ��� *************************/
