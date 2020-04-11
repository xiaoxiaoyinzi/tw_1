/*****************************************************************************
  *文件名: F051test/rtc.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/16
  *描@@述：RTC应用程序
********************************************************************************/
#include "main.h"

char TimeAscii[11];//WYYMMDDHHMM
char TimeAsciiNew[11];//RS485接收到的时间的ASCII码，WYYMMDDHHMM
uint8_t SynchPrediv,AsynchPrediv;//同步、异步预分频值
uint8_t Week,Year,Month,Date,Hours,Minutes,Seconds;//BCD码格式的日期时间
uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//接收到的设置，BCD码格式的日期时间
extern const int8_t tbl_ascii[];//uart_GSM.c中定义

void RTCConfig(void)
{
	uint32_t AsynchPrediv = 0, SynchPrediv = 0;
	RTC_InitTypeDef RTC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//PWR 时钟使能
	//RCC_BackupResetCmd(ENABLE);//ENABLE or DISABLE,备份寄存器,
	PWR_BackupAccessCmd(ENABLE);//允许访问RTC
	RCC_LSEDriveConfig(RCC_LSEDrive_Low);//参数为RCC_LSEDrive_Low或RCC_LSEDrive_MediumLow或RCC_LSEDrive_MediumHigh或RCC_LSEDrive_High
	//无论选择哪个参数，都是外部32.768kHz的时钟，代表驱动强度，根据系统鲁棒性和低功耗要求这种选择
	RCC_LSEConfig(RCC_LSE_ON);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(由OSC32_IN引脚输入)
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){
		__nop();
	}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//配置RTC时钟源为LSE
	SynchPrediv=0xFF;
	AsynchPrediv=0x7F;
	RCC_RTCCLKCmd(ENABLE);//使能RTC时钟
	RTC_WaitForSynchro();//等待RTC APB寄存器同步
	
	RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
	RTC_InitStruct.RTC_SynchPrediv = SynchPrediv;//同步分频值，256-1=255
	RTC_InitStruct.RTC_AsynchPrediv = AsynchPrediv;//异步分频值，128-1=127
	//RTC_StructInit(&RTC_InitStruct);
	RTC_Init(&RTC_InitStruct);
	//RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);
	RTC_ExitInitMode();
}

void ConfigTime(uint8_t Hours,uint8_t Minutes,uint8_t Seconds)
{
	RTC_TimeTypeDef RTC_TimeStructure;
   
	RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours = Hours;
	RTC_TimeStructure.RTC_Minutes = Minutes;
	RTC_TimeStructure.RTC_Seconds = Seconds;
	if( RTC_SetTime( RTC_Format_BCD, &RTC_TimeStructure) == ERROR ){
		__nop();//配置失败处理
	}else{
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);  // 写默认值
	}
 }

void GetTime(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds)
{
	RTC_TimeTypeDef RTC_TimeStructure;
   
	RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure );
	*Hours = RTC_TimeStructure.RTC_Hours;
	*Minutes = RTC_TimeStructure.RTC_Minutes;
	*Seconds = RTC_TimeStructure.RTC_Seconds;
} 
 
void ConfigDate(uint8_t Week,uint8_t Year,uint8_t Month,uint8_t Date)
{
	RTC_DateTypeDef RTC_DateStructure;
   
	RTC_DateStructure.RTC_WeekDay = Week;
	RTC_DateStructure.RTC_Year = Year;
	RTC_DateStructure.RTC_Month = Month;
	RTC_DateStructure.RTC_Date = Date;
	if( RTC_SetDate(RTC_Format_BCD, & RTC_DateStructure) == ERROR )
	{
		__nop();//配置失败处理 
	}
}

void GetDate(uint8_t *Week,uint8_t *Year,uint8_t *Month,uint8_t *Date)
{
	RTC_DateTypeDef RTC_DateStructure;

	RTC_GetDate( RTC_Format_BCD, &RTC_DateStructure );
	*Week = RTC_DateStructure.RTC_WeekDay;
	*Year = RTC_DateStructure.RTC_Year;
	*Month = RTC_DateStructure.RTC_Month;
	*Date = RTC_DateStructure.RTC_Date;
}
void GetTimeAscii(void)//获取时间，填入TimeAscii[11]
{
	GetTime(&Hours,&Minutes,&Seconds);
	GetDate(&Week,&Year,&Month,&Date);
	TimeAscii[0]=tbl_ascii[Week];
	TimeAscii[1]=tbl_ascii[(Year>>4)&0x0F];
	TimeAscii[2]=tbl_ascii[Year&0x0F];
	TimeAscii[3]=tbl_ascii[(Month>>4)&0x0F];
	TimeAscii[4]=tbl_ascii[Month&0x0F];
	TimeAscii[5]=tbl_ascii[(Date>>4)&0x0F];
	TimeAscii[6]=tbl_ascii[Date&0x0F];
	TimeAscii[7]=tbl_ascii[(Hours>>4)&0x0F];
	TimeAscii[8]=tbl_ascii[Hours&0x0F];
	TimeAscii[9]=tbl_ascii[(Minutes>>4)&0x0F];
	TimeAscii[10]=tbl_ascii[Minutes&0x0F];
}
void ConfigRTCAlarm(uint8_t Hours,uint8_t Minutes,uint8_t Seconds)
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
   
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);     // Disable the Alarm A 
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = Hours;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = Minutes;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = Seconds;
   
	//Set the Alarm A 
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
   
	RTC_SetAlarm( RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure );// Configure the RTC Alarm A register 
   
	RTC_ITConfig( RTC_IT_ALRA, ENABLE );// Enable the RTC Alarm A Interrupt
   
	RTC_AlarmCmd( RTC_Alarm_A, ENABLE );//Enable the alarm A
}

void GetRTC_Alarm(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds)
{
	RTC_AlarmTypeDef  RTC_AlarmStructure;
   
	RTC_GetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
	*Hours = RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours;
	*Minutes = RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes;
	*Seconds = RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds;
}

void ConfigRTC_Interrupt(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
   
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;    // Enable the RTC Alarm Interrupt 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus( RTC_IT_ALRA ) != RESET){
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		//报警事件
		__nop();
	} 
}
/************************ (C) 版权 邵开春 *************************/
