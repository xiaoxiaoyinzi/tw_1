/*****************************************************************************
  *�ļ���: F051test/main.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/06
  *��@@������Ӧ�ó���
********************************************************************************/
#include "main.h"

unsigned char FlagLED;//
volatile uint16_t Cnt1ms;//ÿ��1ms
volatile unsigned char Cnt10ms;//��ʱ10���������
volatile uint16_t Cnt1000ms;//��ʱ1000���������
volatile uint16_t Cnt2s;//��ʱ2�������
volatile uint16_t Cnt10s;//��ʱ10�������
volatile uint16_t Cnt60s;//��ʱ60�������
volatile uint16_t Cnt5Minutes;//��ʱ5 ���Ӽ�����
volatile uint16_t Cnt10Minutes;//��ʱ10���Ӽ�����
volatile uint16_t Cnt60Minutes;//��ʱ60���Ӽ�����
volatile uint16_t CntHours;//Сʱ������
volatile uint16_t Cnt72Hours;//72Сʱ����
uint32_t EEdata,data1=0x12345678;

extern volatile uint16_t ADC1ConvData[3];

extern uint16_t NdValue;//����֮�������Ũ��
extern uint8_t BtyValue;//����֮��ĵ�ص�ѹֵ
extern uint8_t SelfStat;//�豸����״̬��bit0-����,bit1-����,bit2-Ƿѹ,bit3-���,bit4-����
extern uint8_t CntADSum;//adֵ�ۼӼ�����
extern uint32_t EEAddrToRead,EEAddrToWrite;
extern uint32_t EEDataReaded,EEDataToWrite;
extern volatile unsigned char  Tx1Read;
extern volatile unsigned char  Tx2Read;
extern double VoltageBty;//����ɵ�ص�ѹֵ

extern float NdZero;//���ADֵ�����ö�̬���
extern uint8_t CntZero;//�������������һ�ν���NdAlarmHandle()��ʼ����������ڵ�6�ε���
extern uint8_t FlagZero;//�����־��δ����
uint8_t FlagPwr;//Ϊ0x55->run,0xAA->poweroff
extern char CircleGPRS[2];//EE��464~465��ASCII����ʽ

void DelayNms(unsigned int t)//ÿ����λ10ms
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
//FlagPwr=0xAA;//δ����״̬
	
	InitRun();//��ʼ��MCU	
	
	while(1)
	{
		//if(FlagPwr==0x55)//RUN MODE
		if(1)//RUN MODE
		{
		//	while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ){ ; }
		//	DMA_ClearFlag(DMA1_FLAG_TC1);// Clear DMA TC flag 
			
			//����AD
			//ReadADCValue();
			//flashtest();
			//if(Tx1Read==4) USART1Trans();//���Դ��ڷ���
			//if(Tx2Read==4) USART2Trans();//���Դ��ڷ���
			
			if(Cnt1000ms<20) GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			else GPIO_SetBits(GPIOB, GPIO_Pin_3);
			if(Cnt1000ms>=1000)//ÿ����ɵĹ���
			{
				Cnt1000ms=0;
				WatDecAlm();//ˮλ��⣬�������
			//GetTimeAscii();//��ȡʱ��
				//ComUDP(1);//�������UDPͨѶ
				Cnt2s++;
				Rs485();//RS485������
				if(Cnt2s>=2){//2��ִ��һ��
					Cnt2s=0;
					//USART1VarInit();//����PA12��ODģʽ���ܹ����5V
					//Rs485();//RS485������
				}
				Cnt10s++;
				if(Cnt10s>=10){//10��ִ��һ��
					Cnt10s=0;
					//ReadADCValue();//��ȡAI1\AI2\ADC_T\ADC_BTY��ֵ
					//CompPT();//ѹ���½��ж�
					//AlmErr10min();//Ũ�ȼ�⡢����
				}
				//1���Ӷ�ʱ
				Cnt60s++;
				if(Cnt60s>=60){//1����
					Cnt60s=0;
					//�����Ӵ�����¼�
					Cnt5Minutes++;
					if(Cnt5Minutes>=5){
						Cnt5Minutes=0;
						//Hart6Hour();//��������
					}
					
					Cnt10Minutes++;
					if(Cnt10Minutes>=10){
						Cnt10Minutes=0;
						//RefreshEE();
						//NVIC_SystemReset();//�����λ
						AlmErr10min();//Ũ�ȼ�⡢����
					}
					//
					//AlmErr10min();//Ũ�ȼ�⡢����,1����һ�Σ�������
					
					Cnt60Minutes++;
					if(Cnt60Minutes>=60){//1Сʱ��ʱ
						Cnt60Minutes=0;
						CntHours++;
						if(CntHours>=6){//6Сʱһ��ͨѶ
						//if(CntHours >= ( Ascii2Hex(CircleGPRS[1])*10+Ascii2Hex(CircleGPRS[1])) ){//2Сʱһ��ͨѶ
							CntHours=0;
							Hart6Hour();//��������
						}
						
						Cnt72Hours++;
						if(Cnt72Hours>=24){//24Сʱ�����λһ��
							Cnt72Hours=0;
							RefreshEE();
							NVIC_SystemReset();//�����λ
						}
					}//1Сʱ����
				}//1���Ӽ���
			}//if(Cnt1000ms>=1000)//ÿ����ɵĹ���

//			
//			ComUDP(1);//����UDP��������Ӧ�����������õ���void ComUDP(uint8_t FlagUDP);
		}else{		//PWROFF MODE
			PwrOff();
		}//
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}//while(1)
}
/************************ (C) ��Ȩ �ۿ��� *************************/
