/*****************************************************************************
  *文件名: FTgprs/main.h 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/01/06
  *描@@述：主应用程序头文件
********************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

//#include "stm32f0xx_gpio.h"
//#include "stm32f0xx.h"
#include "stm32f0xx.h"
#include "math.h"
//LED Control
#define LEDRunOn() GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define LEDRunOff() GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LEDSetOn() GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define LEDSetOff() GPIO_SetBits(GPIOB, GPIO_Pin_4)
//OUTPUT Control
#define VCCTempOn() GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define VCCTempOff() GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define EN5VOn() GPIO_ResetBits(GPIOB, GPIO_Pin_2)//低电平有效
#define EN5VOff() GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define EN4VOn() GPIO_ResetBits(GPIOB, GPIO_Pin_13)//低电平有效
#define EN4VOff() GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define BTY_ENOn() GPIO_ResetBits(GPIOB, GPIO_Pin_12)//低电平有效
#define BTY_ENOff() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define GSMDown() GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define GSMUp() GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define FX485IN() 	GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define FX485OUT() 	GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define EN485() 		GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define DIS485() 		GPIO_SetBits(GPIOA, GPIO_Pin_12)
//INPUT
#define SetEn()      (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 1)//高电平时为设置模式，随时接收输入的设置信息
#define SetDis()     (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 0)//低电平RS485为采集模式，有MCU按时间控制RS485工作
#define PulseAH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 1)//高电平时
#define PulseAL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 0)//低电平
#define PulseBH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 1)//高电平时
#define PulseBL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)//低电平
#define WatH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 1)//高电平时
#define WatL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0)//低电平

#define TranNorm 0
#define TranAlmL 1
#define TranAlmH 2
#define TranErr 3  //传感器故障
#define TranWat 4  //水位，A4000Z:水位报警   A4005Z:水位解除
#define TranBtyLow 5
#define NumAlmHMax 3
#define NumAlmLMax 3
#define NumErrMax 3
#define AlmPointH 50
#define AlmPointL 20

void Reinit_RCC_HSI(void);//重定义系统始时钟为HSI
void Reinit_RCC_HSE(void);//重定义系统始时钟为HSE
void Systick_Init(void);//配置系统滴答中断
void Init_GPIO(void); 
void USART1Init(void);
void CloseUSART1(void);
void USART1VarInit(void);//Usart1辅助变量初始化
void USART1Trans(void);

void Init_ADC(uint8_t ADC_Chanel);
void Init_ADCDMA(void);
void ReadADCValue(void);//读取浓度和电池电压两个通道的AD值
void CompPT(void);//压力下降判断
void NdAlarmHandle(uint16_t AD2611);//浓度、报警、故障判断
void AlmErr10min(void);//10分钟例行报警、故障检测处理程序
void AlmErrWat(void);//水位触发报警、故障检测处理程序
void WatDecAlm(void);//水位报警处理程序
void Hart6Hour(void);//6小时心跳数据上传程序
void InitPWM(uint16_t Freq);//初始化PWM
void SoundOff(void);//关闭声音
void InitRun(void);//开机运行初始化
void InitPwrOff(void);//关机，进入低功耗模式初始化
void DelayNms(unsigned int t);//每个单位10ms
void Delay(unsigned int t);
void InitFlashEE(void);//初始化FlashEE，就是擦除预定的4页
uint32_t FlashEERead(uint32_t addr);//读flash操作
void FlashEEWrite(uint32_t addr,uint32_t data);//写flash操作
void ReadFirstRun(void);//读取初次运行标志,如果是第一次运行，写入默认值并修改初次运行标志
void ReadConfig(void);//读取存储在FlashEE中的配置参数
void ResetFlashEE(uint32_t flashpage);//使用FLASH_ErasePage()擦除flash
void RefreshEE(void);//刷新EE
void NdBiaoding(uint8_t valuebd);//标定操作
void ReadBiaoding(void);//读标定参数


uint8_t Ascii2Hex(uint8_t AsciiValue);//将ASCII码转换成数字
void Init_USART2(void);
void USART2VarInit(void);//Usart2辅助变量初始化
void USART2Trans(void);
void USART2SendData(volatile uint8_t *Data,uint8_t DataLength);//向模块发送数据
void USART2SendStr(volatile char *str);//向模块发送字符串
void CloseUSART2(void);
void turOnGSM(void);
void turOffGSM(void);
void initGSM(void);
uint8_t  UDPLink(void);
uint8_t GPRSHisTrans(uint8_t num);//GPRS发送历史数据
uint8_t GPRSProTrans(uint8_t Flagtrans);//GPRS发送协议
void ComUDP(int8_t FlagUDP);//与服务器UDP通讯
uint8_t SimSendMsgCn(uint8_t *Msg,uint8_t MsgLength);//发中文短信
uint8_t SimSendMsgEn(uint8_t *Msg,uint8_t MsgLength);//发英文短信
void SimRecMsg(uint8_t *BLEData,uint8_t BLEDataLength);//向BLE模块发送命令

void Rs485(void);//检测RS485配置方式
void PwrOff(void);//关机，系统进入低功耗状态

void RTCConfig(void);
void ConfigTime(uint8_t Hours,uint8_t Minutes,uint8_t Seconds);
void GetTime(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds);
void ConfigDate(uint8_t Week,uint8_t Year,uint8_t Month,uint8_t Date);
void GetDate(uint8_t *Week,uint8_t *Year,uint8_t *Month,uint8_t *Date);
void GetTimeAscii(void);//获取时间，填入TimeAscii[11]
void ConfigRTCAlarm(uint8_t Hours,uint8_t Minutes,uint8_t Seconds);
void GetRTC_Alarm(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds);
void ConfigRTC_Interrupt(void);
void RTC_IRQHandler(void);
#endif
/************************ (C) 版权 邵开春 *************************/

