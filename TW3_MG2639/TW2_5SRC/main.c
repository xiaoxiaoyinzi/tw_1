/*****************************************************************************
  *文件名: F051test/main.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/06
  *描@@述：主应用程序
********************************************************************************/
#include "main.h"

unsigned char FlagLED;//
volatile uint16_t Cnt1ms;//每次1ms
volatile unsigned char Cnt10ms;//延时10毫秒计数器
volatile uint16_t Cnt1000ms;//延时1000毫秒计数器
volatile uint16_t Cnt2s;//延时2秒计数器
volatile uint16_t Cnt10s;//延时10秒计数器
volatile uint16_t Cnt60s;//延时60秒计数器
volatile uint16_t Cnt5Minutes;//延时5 分钟计数器
volatile uint16_t Cnt10Minutes;//延时10分钟计数器
volatile uint16_t Cnt60Minutes;//延时60分钟计数器
volatile uint16_t CntHours;//小时计数器
volatile uint16_t Cnt72Hours;//72小时计数
uint32_t EEdata,data1=0x12345678;

extern volatile uint16_t ADC1ConvData[3];

extern uint16_t NdValue;//计算之后的气体浓度
extern uint8_t BtyValue;//计算之后的电池电压值
extern uint8_t SelfStat;//设备自身状态：bit0-报警,bit1-传故,bit2-欠压,bit3-充电,bit4-冲完
extern uint8_t CntADSum;//ad值累加计数器
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;
extern volatile unsigned char  Tx1Read;
extern volatile unsigned char  Tx2Read;
extern double VoltageBty;//换算成电池电压值

extern float NdZero;//零点AD值，采用动态零点
extern uint8_t CntZero;//调零计数器，第一次进入NdAlarmHandle()开始计算次数，在第6次调零
extern uint8_t FlagZero;//调零标志，未调零
uint8_t FlagPwr;//为0x55->run,0xAA->poweroff
extern char CircleGPRS[2];//EE中464~465，ASCII码型式

void DelayNms(unsigned int t)//每个单位10ms
{
	unsigned int i;
	for(i=t;i>0;i--)
	{
		while(Cnt10ms<10){__nop();__nop();__nop();__nop();__nop();}
		Cnt10ms=0;
	}
}
void Delay(unsigned int t)
{
	unsigned int i,j;
	for(i=t;i>0;i--)
		for(j=10;j>0;j--)
			;
}

int main(void)
{
//uint8_t FlagLED;
//FlagPwr=0xAA;//未启动状态
	
	InitRun();//初始化MCU	
	
	while(1)
	{
		//if(FlagPwr==0x55)//RUN MODE
		if(1)//RUN MODE
		{
		//	while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ){ ; }
		//	DMA_ClearFlag(DMA1_FLAG_TC1);// Clear DMA TC flag 
			
			//测试AD
			//ReadADCValue();
			//flashtest();
			//if(Tx1Read==4) USART1Trans();//测试串口发送
			//if(Tx2Read==4) USART2Trans();//测试串口发送
			
			if(Cnt1000ms<20) GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			else GPIO_SetBits(GPIOB, GPIO_Pin_3);
			if(Cnt1000ms>=1000)//每秒完成的工作
			{
				Cnt1000ms=0;
				WatDecAlm();//水位检测，触发检测
			//GetTimeAscii();//获取时钟
				//ComUDP(1);//与服务器UDP通讯
				Cnt2s++;
				Rs485();//RS485处理函数
				if(Cnt2s>=2){//2秒执行一次
					Cnt2s=0;
					//USART1VarInit();//测试PA12在OD模式下能够输出5V
					//Rs485();//RS485处理函数
				}
				Cnt10s++;
				if(Cnt10s>=10){//10秒执行一次
					Cnt10s=0;
					//ReadADCValue();//读取AI1\AI2\ADC_T\ADC_BTY的值
					//CompPT();//压力下降判断
					//AlmErr10min();//浓度检测、处理
				}
				//1分钟定时
				Cnt60s++;
				if(Cnt60s>=60){//1分钟
					Cnt60s=0;
					//按分钟处理的事件
					Cnt5Minutes++;
					if(Cnt5Minutes>=5){
						Cnt5Minutes=0;
						//Hart6Hour();//心跳程序
					}
					
					Cnt10Minutes++;
					if(Cnt10Minutes>=10){
						Cnt10Minutes=0;
						//RefreshEE();
						//NVIC_SystemReset();//软件复位
						AlmErr10min();//浓度检测、处理
					}
					//
					//AlmErr10min();//浓度检测、处理,1分钟一次，调试用
					
					Cnt60Minutes++;
					if(Cnt60Minutes>=60){//1小时定时
						Cnt60Minutes=0;
						CntHours++;
						if(CntHours>=6){//6小时一次通讯
						//if(CntHours >= ( Ascii2Hex(CircleGPRS[1])*10+Ascii2Hex(CircleGPRS[1])) ){//2小时一次通讯
							CntHours=0;
							Hart6Hour();//心跳程序
						}
						
						Cnt72Hours++;
						if(Cnt72Hours>=24){//24小时软件复位一次
							Cnt72Hours=0;
							RefreshEE();
							NVIC_SystemReset();//软件复位
						}
					}//1小时计数
				}//1分钟计数
			}//if(Cnt1000ms>=1000)//每秒完成的工作

//			
//			ComUDP(1);//测试UDP程序，正常应根据周期设置调用void ComUDP(uint8_t FlagUDP);
		}else{		//PWROFF MODE
			PwrOff();
		}//
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}//while(1)
}
/************************ (C) 版权 邵开春 *************************/
