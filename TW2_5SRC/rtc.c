/*****************************************************************************
  *�ļ���: F051test/rtc.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/16
  *��@@����RTCӦ�ó���
********************************************************************************/
#include "main.h"

char TimeAscii[11];//WYYMMDDHHMM
char TimeAsciiNew[11];//RS485���յ���ʱ���ASCII�룬WYYMMDDHHMM
uint8_t SynchPrediv,AsynchPrediv;//ͬ�����첽Ԥ��Ƶֵ
uint8_t Week,Year,Month,Date,Hours,Minutes,Seconds;//BCD���ʽ������ʱ��
uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//���յ������ã�BCD���ʽ������ʱ��
extern const int8_t tbl_ascii[];//uart_GSM.c�ж���

void RTCConfig(void)
{
	uint32_t AsynchPrediv = 0, SynchPrediv = 0;
	RTC_InitTypeDef RTC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//PWR ʱ��ʹ��
	//RCC_BackupResetCmd(ENABLE);//ENABLE or DISABLE,���ݼĴ���,
	PWR_BackupAccessCmd(ENABLE);//�������RTC
	RCC_LSEDriveConfig(RCC_LSEDrive_Low);//����ΪRCC_LSEDrive_Low��RCC_LSEDrive_MediumLow��RCC_LSEDrive_MediumHigh��RCC_LSEDrive_High
	//����ѡ���ĸ������������ⲿ32.768kHz��ʱ�ӣ���������ǿ�ȣ�����ϵͳ³���Ժ͵͹���Ҫ������ѡ��
	RCC_LSEConfig(RCC_LSE_ON);//RCC_LSE_OFF/RCC_LSE_On/RCC_LSE_Bypass(��OSC32_IN��������)
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){
		__nop();
	}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//����RTCʱ��ԴΪLSE
	SynchPrediv=0xFF;
	AsynchPrediv=0x7F;
	RCC_RTCCLKCmd(ENABLE);//ʹ��RTCʱ��
	RTC_WaitForSynchro();//�ȴ�RTC APB�Ĵ���ͬ��
	
	RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
	RTC_InitStruct.RTC_SynchPrediv = SynchPrediv;//ͬ����Ƶֵ��256-1=255
	RTC_InitStruct.RTC_AsynchPrediv = AsynchPrediv;//�첽��Ƶֵ��128-1=127
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
		__nop();//����ʧ�ܴ���
	}else{
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);  // дĬ��ֵ
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
		__nop();//����ʧ�ܴ��� 
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
void GetTimeAscii(void)//��ȡʱ�䣬����TimeAscii[11]
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
		//�����¼�
		__nop();
	} 
}
/************************ (C) ��Ȩ �ۿ��� *************************/
