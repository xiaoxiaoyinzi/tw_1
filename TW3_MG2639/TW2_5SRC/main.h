/*****************************************************************************
  *�ļ���: FTgprs/main.h 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/01/06
  *��@@������Ӧ�ó���ͷ�ļ�
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
#define EN5VOn() GPIO_ResetBits(GPIOB, GPIO_Pin_2)//�͵�ƽ��Ч
#define EN5VOff() GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define EN4VOn() GPIO_ResetBits(GPIOB, GPIO_Pin_13)//�͵�ƽ��Ч
#define EN4VOff() GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define BTY_ENOn() GPIO_ResetBits(GPIOB, GPIO_Pin_12)//�͵�ƽ��Ч
#define BTY_ENOff() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define GSMDown() GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define GSMUp() GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define FX485IN() 	GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define FX485OUT() 	GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define EN485() 		GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define DIS485() 		GPIO_SetBits(GPIOA, GPIO_Pin_12)
//INPUT
#define SetEn()      (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 1)//�ߵ�ƽʱΪ����ģʽ����ʱ���������������Ϣ
#define SetDis()     (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 0)//�͵�ƽRS485Ϊ�ɼ�ģʽ����MCU��ʱ�����RS485����
#define PulseAH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 1)//�ߵ�ƽʱ
#define PulseAL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 0)//�͵�ƽ
#define PulseBH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 1)//�ߵ�ƽʱ
#define PulseBL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)//�͵�ƽ
#define WatH()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 1)//�ߵ�ƽʱ
#define WatL()    (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0)//�͵�ƽ

#define TranNorm 0
#define TranAlmL 1
#define TranAlmH 2
#define TranErr 3  //����������
#define TranWat 4  //ˮλ��A4000Z:ˮλ����   A4005Z:ˮλ���
#define TranBtyLow 5
#define NumAlmHMax 3
#define NumAlmLMax 3
#define NumErrMax 3
#define AlmPointH 50
#define AlmPointL 20

void Reinit_RCC_HSI(void);//�ض���ϵͳʼʱ��ΪHSI
void Reinit_RCC_HSE(void);//�ض���ϵͳʼʱ��ΪHSE
void Systick_Init(void);//����ϵͳ�δ��ж�
void Init_GPIO(void); 
void USART1Init(void);
void CloseUSART1(void);
void USART1VarInit(void);//Usart1����������ʼ��
void USART1Trans(void);

void Init_ADC(uint8_t ADC_Chanel);
void Init_ADCDMA(void);
void ReadADCValue(void);//��ȡŨ�Ⱥ͵�ص�ѹ����ͨ����ADֵ
void CompPT(void);//ѹ���½��ж�
void NdAlarmHandle(uint16_t AD2611);//Ũ�ȡ������������ж�
void AlmErr10min(void);//10�������б��������ϼ�⴦�����
void AlmErrWat(void);//ˮλ�������������ϼ�⴦�����
void WatDecAlm(void);//ˮλ�����������
void Hart6Hour(void);//6Сʱ���������ϴ�����
void InitPWM(uint16_t Freq);//��ʼ��PWM
void SoundOff(void);//�ر�����
void InitRun(void);//�������г�ʼ��
void InitPwrOff(void);//�ػ�������͹���ģʽ��ʼ��
void DelayNms(unsigned int t);//ÿ����λ10ms
void Delay(unsigned int t);
void InitFlashEE(void);//��ʼ��FlashEE�����ǲ���Ԥ����4ҳ
uint32_t FlashEERead(uint32_t addr);//��flash����
void FlashEEWrite(uint32_t addr,uint32_t data);//дflash����
void ReadFirstRun(void);//��ȡ�������б�־,����ǵ�һ�����У�д��Ĭ��ֵ���޸ĳ������б�־
void ReadConfig(void);//��ȡ�洢��FlashEE�е����ò���
void ResetFlashEE(uint32_t flashpage);//ʹ��FLASH_ErasePage()����flash
void RefreshEE(void);//ˢ��EE
void NdBiaoding(uint8_t valuebd);//�궨����
void ReadBiaoding(void);//���궨����


uint8_t Ascii2Hex(uint8_t AsciiValue);//��ASCII��ת��������
void Init_USART2(void);
void USART2VarInit(void);//Usart2����������ʼ��
void USART2Trans(void);
void USART2SendData(volatile uint8_t *Data,uint8_t DataLength);//��ģ�鷢������
void USART2SendStr(volatile char *str);//��ģ�鷢���ַ���
void CloseUSART2(void);
void turOnGSM(void);
void turOffGSM(void);
void initGSM(void);
uint8_t  UDPLink(void);
uint8_t GPRSHisTrans(uint8_t num);//GPRS������ʷ����
uint8_t GPRSProTrans(uint8_t Flagtrans);//GPRS����Э��
void ComUDP(int8_t FlagUDP);//�������UDPͨѶ
uint8_t SimSendMsgCn(uint8_t *Msg,uint8_t MsgLength);//�����Ķ���
uint8_t SimSendMsgEn(uint8_t *Msg,uint8_t MsgLength);//��Ӣ�Ķ���
void SimRecMsg(uint8_t *BLEData,uint8_t BLEDataLength);//��BLEģ�鷢������

void Rs485(void);//���RS485���÷�ʽ
void PwrOff(void);//�ػ���ϵͳ����͹���״̬

void RTCConfig(void);
void ConfigTime(uint8_t Hours,uint8_t Minutes,uint8_t Seconds);
void GetTime(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds);
void ConfigDate(uint8_t Week,uint8_t Year,uint8_t Month,uint8_t Date);
void GetDate(uint8_t *Week,uint8_t *Year,uint8_t *Month,uint8_t *Date);
void GetTimeAscii(void);//��ȡʱ�䣬����TimeAscii[11]
void ConfigRTCAlarm(uint8_t Hours,uint8_t Minutes,uint8_t Seconds);
void GetRTC_Alarm(uint8_t *Hours,uint8_t *Minutes,uint8_t *Seconds);
void ConfigRTC_Interrupt(void);
void RTC_IRQHandler(void);
#endif
/************************ (C) ��Ȩ �ۿ��� *************************/

