/*****************************************************************************
  *文件名: F051test/Init.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/06
  *描@@述：初始化
********************************************************************************/
#include "main.h"

uint8_t FlagRCC;////0xAA->HSI,//0x55->HSE

extern uint8_t FlagPwr;//为0x55->run,0xAA->poweroff
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;
extern uint8_t EEArray[];//对应488个模拟EE地址

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0;
//定义FlashEE数据
uint8_t FlagFirstRun=0;

char APN[20];//EE中400~419，ASCII码型式
char USER[12];//EE中420~431，ASCII码型式
char PWD[8];//EE中432~439，ASCII码型式
char IPAddr[16];//EE中440~455，ASCII码型式
char PortID[4];//EE中456~459，ASCII码型式
char CircleAD[2];//EE中460~461，ASCII码型式
char CircleGPRS[2];//EE中464~465，ASCII码型式
char DeviceID[12];//EE中468~479设备识别号,共11字节有效，12=4*3，DeviceID[11]读写Flash时溢出
extern volatile uint32_t FTgklj,FTgkss;//工况累计流量,存储在EE484-487;工况瞬时流量,在EE中不存储
extern volatile unsigned char FlagFTgklj;//1:精确到0.1m3，2:精确到1m3
extern volatile unsigned char NumPulseAL;//PulseAL的脉冲数

extern char TimeAscii[11];//rtc.c定义,时间的ASCII码，WYYMMDDHHMM
//extern uint8_t Week,Year,Month,Date,Hours,Minutes,Seconds;//rtc.c定义,BCD码格式的日期时间
//extern const int8_t tbl_ascii[];//uart_GSM.c中定义
//uint16_t Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
const char DeviceIDDefault[12]={'0','5','3','1','0','0','0','0','0','0','4','\0'};//05310000001
const char APNDefault[20]={'c','m','n','e','t','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};//cmnet
const char USERDefault[12]={'U','S','E','R','\0','\0','\0','\0','\0','\0','\0','\0'};//USER
const char PWDDefault[8]={'P','W','D','\0','\0','\0','\0','\0'};//USER
//const char IPAddrDefault[16]={'5','8','.','5','6','.','2','4','7','.','6','6','\0','\0','\0','\0'};//58.56.247.66
const char IPAddrDefault[16]={'1','2','4','.','1','2','8','.','2','0','0','.','3','0','\0','\0'};//124.128.200.30
const char PortIDDefault[4]={'5','2','2','1'};//5007
const char CircleADDefault[2]={'0','5'};//0x05,单位分钟
const char CircleGPRSDefault[2]={'0','1'};//0x01，单位小时

extern char FTgkljAscii[];//ADC_Calcu.c中定义
extern const int8_t tbl_ascii[];//uart_GSM.c中定义

void Reinit_RCC_HSI(void)//重定义系统始时钟为HSI
{	
	FlagRCC=0xAA;//0xAA->HSI
	RCC_HSEConfig(RCC_HSE_OFF);//RCC_HSE_OFF/RCC_HSE_ON/RCC_HSE_Bypass(由OSC_IN引脚输入)
	//RCC_WaitForHSEStartUp();//返回SUCCESS或ERROR
	
	RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSI时钟校准数值，无需更改
	RCC_HSICmd(ENABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14时钟校准数值，无需更改
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14为ADC专用振荡器
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	//RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(由OSC32_IN引脚输入)
	//RCC_LSEDriveConfig(RCC_LSEDrive_MediumLow);//参数为RCC_LSEDrive_Low或RCC_LSEDrive_MediumLow或RCC_LSEDrive_MediumHigh或RCC_LSEDrive_High
	//无论选择哪个参数，都是外部32.768kHz的时钟，代表驱动强度，根据系统鲁棒性和低功耗要求这种选择
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4);//参数1取值RCC_PLLSource_HSI_Div2或RCC_PLLSource_PREDIV1（HSE的预分频）
	//参数2为PLL倍频数RCC_PLLMul_2到RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,设置PLL预分频数，仅在PLL禁止时改�
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE
	

	//RCC_ClockSecuritySystemCmd(DISABLE);//ENABLE or DISABLE,开关时钟安全检测系统
	//RCC_MCOConfig(RCC_MCOSource_LSI);//设置MCO时钟输出脚的时钟源,RCC_MCOSource_**,
	//**为NoClock/HSI14/LSI/LSE/HSI/HSE/SYSCLK/PLLCLK_Div2
	//RCC_MCOConfig(RCC_MCOSource_LSI,RCC_MCOPrescaler_2);//参数2取值为RCC_MCOPrescaler_**,**为(1/2/4/8/16/32/64/128)
	
	//配置系统时钟为PLL
	//while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//等待PLL稳定
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	//while(RCC_GetSYSCLKSource()!=0x08);//0x00:HSI, 0x04:HSE, 0x08:PLL
	//配置系统时钟为HSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x00);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	RCC_HCLKConfig(RCC_SYSCLK_Div16);//AHB总线(core,DMA)时钟,RCC_SYSCLK_Div*,*为1,2,4,8,16,64,128,256,512
	RCC_PCLKConfig(RCC_HCLK_Div1);//APB总线(外设)时钟,RCC_HCLK_Div*,*为1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_LSE);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,两个参数都是32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	//RCC_RTCCLKCmd(DISABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,备份寄存器,
	
	//RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
	//参数1：RCC_AHBPeriph_GPIOA~F，RCC_AHBPeriph_TS，RCC_AHBPeriph_CRC,RCC_AHBPeriph_FLITF,RCC_AHBPeriph_SRAM,RCC_AHBPeriph_DMA1
	
	//RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//参数1：RCC_APB2Periph_SYSCFG,RCC_APB2Periph_ADC1,RCC_APB2Periph_TIM1,RCC_APB2Periph_SPI1,
	//RCC_APB2Periph_USART1,RCC_APB2Periph_TIM15~7,RCC_APB2Periph_DBGMCU
	
	//RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//参数1：RCC_APB1Periph_TIM2/3/6/14,RCC_APB1Periph_WWDG,RCC_APB1Periph_SPI2,RCC_APB1Periph_USART2,
	//RCC_APB1Periph_I2C1~2,RCC_APB1Periph_PWR,RCC_APB1Periph_ADC,RCC_APB1Periph_CEC
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//中断使能,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,
	//RCC_IT_PLLRDY,RCC_IT_HSI14RDY,
	//FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);//stm32f0xx_rcc.c第1473行
	//RCC_ClearFlag();//清除reset flags
	//ITStatus RCC_GetITStatus(uint8_t RCC_IT);//stm32f0xx_rcc.c第1544行
	//RCC_ClearITPendingBit(uint8_t RCC_IT);//stm32f0xx_rcc.c第1577行
	
}
void Reinit_RCC_HSE(void)//重定义系统始时钟HSE
{	
	FlagRCC=0x55;//0x55->HSE
	RCC_HSEConfig(RCC_HSE_ON);//RCC_HSE_OFF/RCC_HSE_ON/RCC_HSE_Bypass(由OSC_IN引脚输入)
	RCC_WaitForHSEStartUp();//返回SUCCESS或ERROR
	
	//RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSI时钟校准数值，无需更改
	//RCC_HSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14时钟校准数值，无需更改
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14为ADC专用振荡器
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	//RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(由OSC32_IN引脚输入)
	//RCC_LSEDriveConfig(RCC_LSEDrive_High);//参数为RCC_LSEDrive_Low或RCC_LSEDrive_MediumLow或RCC_LSEDrive_MediumHigh或RCC_LSEDrive_High
	//无论选择哪个参数，都是外部32.768kHz的时钟，代表驱动强度，根据系统鲁棒性和低功耗要求这种选择
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8);//参数1取值RCC_PLLSource_HSI_Div2或RCC_PLLSource_PREDIV1（HSE的预分频）
	//参数2为PLL倍频数RCC_PLLMul_2到RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,设置PLL预分频数，仅在PLL禁止时改�
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE
	

	//RCC_ClockSecuritySystemCmd(DISABLE);//ENABLE or DISABLE,开关时钟安全检测系统
	//RCC_MCOConfig(RCC_MCOSource_LSI);//设置MCO时钟输出脚的时钟源,RCC_MCOSource_**,
	//**为NoClock/HSI14/LSI/LSE/HSI/HSE/SYSCLK/PLLCLK_Div2
	//RCC_MCOConfig(RCC_MCOSource_LSI,RCC_MCOPrescaler_2);//参数2取值为RCC_MCOPrescaler_**,**为(1/2/4/8/16/32/64/128)
	
	//配置系统时钟为PLL
	//while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//等待PLL稳定
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	//while(RCC_GetSYSCLKSource()!=0x08);//0x00:HSI, 0x04:HSE, 0x08:PLL
	//配置系统时钟为HSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x04);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB总线(core,DMA)时钟,RCC_SYSCLK_Div*,*为1,2,4,8,16,64,128,256,512
	RCC_PCLKConfig(RCC_HCLK_Div1);//APB总线(外设)时钟,RCC_HCLK_Div*,*为1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_LSE);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,两个参数都是32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	//RCC_RTCCLKCmd(ENABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,备份寄存器,
	
	//RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
	//参数1：RCC_AHBPeriph_GPIOA~F，RCC_AHBPeriph_TS，RCC_AHBPeriph_CRC,RCC_AHBPeriph_FLITF,RCC_AHBPeriph_SRAM,RCC_AHBPeriph_DMA1
	
	//RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//参数1：RCC_APB2Periph_SYSCFG,RCC_APB2Periph_ADC1,RCC_APB2Periph_TIM1,RCC_APB2Periph_SPI1,
	//RCC_APB2Periph_USART1,RCC_APB2Periph_TIM15~7,RCC_APB2Periph_DBGMCU
	
	//RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);//在相应的外设初始化中修改,
	//参数1：RCC_APB1Periph_TIM2/3/6/14,RCC_APB1Periph_WWDG,RCC_APB1Periph_SPI2,RCC_APB1Periph_USART2,
	//RCC_APB1Periph_I2C1~2,RCC_APB1Periph_PWR,RCC_APB1Periph_ADC,RCC_APB1Periph_CEC
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//中断使能,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,
	//RCC_IT_PLLRDY,RCC_IT_HSI14RDY,
	//FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);//stm32f0xx_rcc.c第1473行
	//RCC_ClearFlag();//清除reset flags
	//ITStatus RCC_GetITStatus(uint8_t RCC_IT);//stm32f0xx_rcc.c第1544行
	//RCC_ClearITPendingBit(uint8_t RCC_IT);//stm32f0xx_rcc.c第1577行
	
}
void Systick_Init(void)
{
	if(FlagRCC==0xAA) 
		SysTick_Config(500);//系统时钟为HSI(8MHz)的16分频,1000个SYSCLK周期刚好是1ms
	else if(FlagRCC==0x55)
		SysTick_Config(4000);//系统时钟为HSE(4MHz)的4分频,1000个SYSCLK周期刚好是1ms
//	SysTick_Config(40000);//系统时钟为PLL:40MHz,40000个SYSCLK周期刚好是1ms
}

void Init_GPIO(void) 
{
	//*
	GPIO_InitTypeDef GPIO_InitStruct; //定义结构体
	
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
	VCCTempOff();//关闭LMT84
	LEDSetOff();
	LEDRunOff();
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11;//pulseA,pulseB
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	
	//配置悬空脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_15;//空
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
	GPIO_SetBits(GPIOB, GPIO_Pin_13);//EN4V,低电平有效
	GPIO_SetBits(GPIOB, GPIO_Pin_14);//GPRSON,低电平有效
	
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

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;//PA8检测RS485工作模式，应放在Init_GPIO()中,循环检测
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_15;//空
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	
	//SWD口，正式生产时启用，调试时屏蔽
	/*
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;//空
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_13);
	GPIO_ResetBits(GPIOA, GPIO_Pin_14);
	*/
	
	//初始化RS485，调试用程序段，调试好后关闭
	//USART1Init();
	//USART1VarInit();
	CloseUSART1();
}
void ReadFirstRun(void)//读取初次运行标志,如果是第一次运行，写入默认值并修改初次运行标志
{
	uint32_t EEData;
	uint8_t i;
	//uint8_t FlagFirstRun=0;
	EEData=FlashEERead(480);
	FlagFirstRun=(uint8_t)(EEData & 0x000000FF);//480
	EEArray[480]=FlagFirstRun;
	FlagFTgklj=(uint8_t)((EEData>>8) & 0x000000FF);//481
	if(FlagFirstRun!=1){//第一次运行，将默认值写入EE,"05310000001'\0'"
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
		//修改初次运行标志
		FlashEEWrite(480,0x00000101);//已运行过，0.1m3/h
		EEArray[480]=0x01;//防止修改时，将0xFF写入EE中480位置
		EEArray[481]=0x01;//写入FlagFTgklj
		FlashEEWrite(484,0x00000000);//第一次运行，累积量为0 
		FTgklj=0;
		EEArray[484]=0x00;EEArray[485]=0x00;EEArray[486]=0x00;EEArray[487]=0x00;//将0写入EE中484位置
	}else{
		__nop();
	}
}
void ReadConfig(void)//读取存储在FlashEE中的配置参数
{
	uint8_t Index;
	uint32_t EEdata;
	//char DeviceID[12];//EE中400~419，ASCII码型式
	//char APN[20];//EE中400~419，ASCII码型式
	//char USER[12];//EE中420~431，ASCII码型式
	//char PWD[8];//EE中432~439，ASCII码型式
	//char IPAddr[16];//EE中440~455，ASCII码型式
	//char PortID[4];//EE中456~459，ASCII码型式
	//char CircleAD[2];//EE中460~461，ASCII码型式
	//char CircleGPRS[2];//EE中464~465，ASCII码型式
	for(Index=0;Index<20;Index+=4){
		EEdata=FlashEERead(400+Index);
		APN[Index]=(char)(EEdata & 0x000000FF);
		APN[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		APN[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		APN[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	APN[19]='\0';//末字节添加'\0',初次上电时，参数未经过配置，防止GPRS发送时出错
	for(Index=0;Index<20;Index++)
		EEArray[400+Index]=APN[Index];
	
	for(Index=0;Index<12;Index+=4){
		EEdata=FlashEERead(420+Index);
		USER[Index]=(char)(EEdata & 0x000000FF);
		USER[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		USER[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		USER[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	USER[11]='\0';//末字节添加'\0',初次上电时，参数未经过配置，防止GPRS发送时出错
	for(Index=0;Index<12;Index++)
		EEArray[420+Index]=USER[Index];
	
	for(Index=0;Index<8;Index+=4){
		EEdata=FlashEERead(432+Index);
		PWD[Index]=(char)(EEdata & 0x000000FF);
		PWD[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		PWD[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		PWD[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	PWD[7]='\0';//末字节添加'\0',初次上电时，参数未经过配置，防止GPRS发送时出错
	for(Index=0;Index<8;Index++)
		EEArray[432+Index]=PWD[Index];
	
	for(Index=0;Index<16;Index+=4){
		EEdata=FlashEERead(440+Index);
		IPAddr[Index]=(char)(EEdata & 0x000000FF);
		IPAddr[Index+1]=(char)((EEdata>>8) & 0x000000FF);
		IPAddr[Index+2]=(char)((EEdata>>16) & 0x000000FF);
		IPAddr[Index+3]=(char)((EEdata>>24) & 0x000000FF);
	}
	IPAddr[15]='\0';//末字节添加'\0',初次上电时，参数未经过配置，防止GPRS发送时出错
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
	DeviceID[11]='\0';//末字节添加'\0',初次上电时，参数未经过配置，防止GPRS发送时出错
	for(Index=0;Index<12;Index++){
		EEArray[468+Index]=DeviceID[Index];
	}
	EEdata=FlashEERead(480);
	EEArray[480]=(char)(EEdata & 0x000000FF);
	FlagFTgklj=(char)((EEdata>>8) & 0x000000FF);
	EEArray[481]=FlagFTgklj;
	EEdata=FlashEERead(484);
	FTgklj=EEdata & 0x00FFFFFF;//累计流量整数占 3 字节
	NumPulseAL = (EEdata>>24) & 0xFF;////累计流量整数占 1 字节
	EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
	EEArray[486]=(FTgklj>>16) & 0xFF;EEArray[487]=NumPulseAL;

	//FTgkljAscii[1]=tbl_ascii[(char)((FTgklj>>28)&0x0F)];FTgkljAscii[2]=tbl_ascii[(char)((FTgklj>>24)&0x0F)];
	FTgkljAscii[1]='0';FTgkljAscii[2]='0';
	FTgkljAscii[3]=tbl_ascii[(char)((FTgklj>>20)&0x0F)];FTgkljAscii[4]=tbl_ascii[(char)((FTgklj>>16)&0x0F)];
	FTgkljAscii[5]=tbl_ascii[(char)((FTgklj>>12)&0x0F)];FTgkljAscii[6]=tbl_ascii[(char)((FTgklj>>8)&0x0F)];
	FTgkljAscii[7]=tbl_ascii[(char)((FTgklj>>4)&0x0F)];FTgkljAscii[8]=tbl_ascii[(char)(FTgklj&0x0F)];
	
	ReadBiaoding();//读EE中标定参数
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

	//InitFlashEE();//清空4页FlashEE
	ReadConfig();//读取存储在FlashEE中的配置参数//
	ReadFirstRun();
	ReadConfig();//读取存储在FlashEE中的配置参数//

	RTCConfig();//开机初始化RTC
	GetTimeAscii();

}

void InitPwrOff(void)//关机，进入低功耗模式初始化
{
	//GPIO_InitTypeDef GPIO_InitStruct; //定义结构体
	/*
	RCC_HSEConfig(RCC_HSE_OFF);
	RCC_AdjustHSICalibrationValue(0x10);//0~0x1F,HSI时钟校准数值，无需更改
	RCC_HSICmd(ENABLE);//ENABLE or DISABLE
	
	RCC_AdjustHSI14CalibrationValue(0);//0~0x1F,HSI14时钟校准数值，无需更改
	RCC_HSI14Cmd(DISABLE);//ENABLE or DISABLE,HSI14为ADC专用振荡器
	RCC_HSI14ADCRequestCmd(DISABLE);//ENABLE or DISABLE
	
	RCC_LSEConfig(RCC_LSE_OFF);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(由OSC32_IN引脚输入)
	RCC_LSEDriveConfig(RCC_LSEDrive_Low);//参数为RCC_LSEDrive_Low或RCC_LSEDrive_MediumLow或RCC_LSEDrive_MediumHigh或RCC_LSEDrive_High
	//无论选择哪个参数，都是外部32.768kHz的时钟，代表驱动强度，根据系统鲁棒性和低功耗要求这种选择
	
	RCC_LSICmd(DISABLE);//ENABLE or DISABLE
	
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_10);//参数1取值RCC_PLLSource_HSI_Div2或RCC_PLLSource_PREDIV1（HSE的预分频）
	//参数2为PLL倍频数RCC_PLLMul_2到RCC_PLLMul_16
	RCC_PREDIV1Config(RCC_PREDIV1_Div1);//RCC_PREDIV1_Div1~RCC_PREDIV1_Div16,设置PLL预分频数，仅在PLL禁止时改�
	RCC_PLLCmd(DISABLE);//ENABLE or DISABLE

	//配置系统时钟为HSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//RCC_SYSCLKSource_HSI/RCC_SYSCLKSource_HSE/RCC_SYSCLKSource_PLLCLK
	while(RCC_GetSYSCLKSource()!=0x00);//0x00:HSI, 0x04:HSE, 0x08:PLL
	
	//RCC_HCLKConfig(RCC_SYSCLK_Div1);//15625Hz.AHB总线(core,DMA)时钟,RCC_SYSCLK_Div*,*为1,2,4,8,16,64,128,256,512
	FLASH_PrefetchBufferCmd(ENABLE);//RCC_SYSCLK_Div大于1，首先使能Flash预读寄存器
	RCC_HCLKConfig(RCC_SYSCLK_Div64);//15625Hz.AHB总线(core,DMA)时钟,RCC_SYSCLK_Div*,*为1,2,4,8,16,64,128,256,512
	//RCC_PCLKConfig(RCC_HCLK_Div1);//APB总线(外设)时钟,RCC_HCLK_Div*,*为1,2,4,8,16
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);//RCC_ADCCLK_PCLK_Div2,RCC_ADCCLK_PCLK_Div4,RCC_ADCCLK_HSI14
	RCC_CECCLKConfig(RCC_CECCLK_HSI_Div244);//RCC_CECCLK_HSI_Div244,RCC_CECCLK_LSE,两个参数都是32768Hz
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);//RCC_I2C1CLK_HSI,RCC_I2C1CLK_SYSCLK
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);//RCC_USART1CLK_PCLK,RCC_USART1CLK_SYSCLK,RCC_USART1CLK_LSE,RCC_USART1CLK_HSI
	//RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);//
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);//RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_LSI,RCC_RTCCLKSource_HSE_Div32
	RCC_RTCCLKCmd(DISABLE);//ENABLE or DISABLE
	RCC_BackupResetCmd(DISABLE);//ENABLE or DISABLE,备份寄存器,
	
	RCC_ITConfig(RCC_IT_LSIRDY,DISABLE);//中断使能,RCC_IT_LSIRDY,RCC_IT_LSERDY,RCC_IT_HSIRDY,RCC_IT_HSERDY,

	SysTick_Config(15);//系统时钟为HSI:8MHz,8000个SYSCLK周期刚好是1ms
	//SysTick_Config(8000);//系统时钟为HSI:8MHz,8000个SYSCLK周期刚好是1ms
	*/
	//除GPIOA\GPIOB外关闭所有外设
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);//ADC时钟使能
	ADC_Cmd(ADC1, DISABLE); //关闭ADC1   
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);//DMA时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE ); //usart1总线关闭
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
/************************ (C) 版权 邵开春 *************************/
