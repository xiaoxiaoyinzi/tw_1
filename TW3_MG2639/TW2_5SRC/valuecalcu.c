#include "main.h"
#include "math.h"

//#define VoltageRef 3300 //�ο���ѹΪ3.3V
#define VoltageRef 2500 //�ο���ѹΪ2.5V
#define VoltageSensor 2500 //�������źŲ��ֹ����ѹ2.5V
#define ResistorLoad 3300 //���ص���3300ŷķ�����ص����VoltageSensor
#define LineB 3.699;//��ʽlg(c)=k*lg(Rs/R0)+b�е�bֵ

#define BAT_CHG_ON()       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 0)//���ڳ��
#define BAT_CHG_OFF()        (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 1)//δ���
#define BAT_PG_ON()       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 0)//BQ24041�����ѹ����
#define BAT_PG_OFF()        (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 1)//BQ24041�����ѹ�쳣

uint16_t DeltaAVCOValue;//��ǰ���ڵ�AVCOValue��ǰһ���ڵĲ�ֵ
uint16_t DeltaAVCOValuePrev;//ǰһ���ڵ�AVCOValue��ǰ�����ڵĲ�ֵ
double VoltageBty;//����ɵ�ص�ѹֵ
double VoltageRs;//Rs�ϵķ�ѹ��Rs��GND,RL��2.5V
double ResistorSensor;//����������Rs
const double LineK[25]={0,-2.06327,-2.05828,-2.06409,-2.06241,-2.05984,-2.08674,-2.07243,-2.05984,//0-8
                          -2.05984,-2.07441,-2.03392,-2.08396,-2.0817, -2.06476,-2.06881,-2.05984,//9-18
                          -2.05616,-2.06656,-2.0568, -2.06539,-2.06235,-2.06212,-2.06817,-2.06551};//��ʽlg(c)=k*lg(Rs/R0)+b�е�kֵ;
uint8_t IndexSensor=13;//б��ֵ�ʹ�����R0ֵ�������ţ�Ĭ��Ϊ#14���������궨ʱ�޸�
const uint16_t Resistor0[25]={0,715,787,866,953,1050,1150,1270,1400,1540,1690,1870,2050,//0-12,//��������5000ppm�����еĵ���ֵ
                           2260,2490,2740,3010,3320,3650,4020,4420,4870,4360,5900,6490};//13-24		
const uint16_t Resistor1000[25]={0,1560,1720,1889,2080,2294,2487,2761,3058,3364,3672,4126,4438,//0-12,//��������1000ppm�����еĵ���ֵ
                           4897,5429,5965,6575,7263,7953,8792,9635,10628,11699,12848,14147};//13-24		
double VaulueX;//Y=K*X+b�е�Xֵ��Ҳ����lg(Rs/R0)��ֵ
double VaulueY;//Y=K*X+b�е�Yֵ��Ҳ����lg(c)��ֵ													 
double COValueTemp1;//��ʱŨ�ȵ�ֵ
float NdZero;//���ADֵ�����ö�̬���
uint8_t CntZero=0;//�������������һ�ν���NdAlarmHandle()��ʼ����������ڵ�6�ε���
uint8_t FlagZero=0xAA;//�����־��δ����
													 											 
volatile uint16_t ADBtyValue,ADCOValue,ADTempValue;//����ת����ֵ
volatile uint32_t SumBtyValue,SumCOValue,SumTempValue;//64��ת����ֵ
volatile uint16_t AVBtyValue,AVCOValue,AVTempValue;//64��ת����ƽ��ֵ
uint16_t COValue;//����֮�������Ũ��
uint8_t BtyValue;//����֮��ĵ�ص�ѹֵ
uint8_t SelfStat=0;//�豸����״̬��bit0-����,bit1-����,bit2-Ƿѹ,bit3-���,bit4-����
uint8_t CntADSum=0;//adֵ�ۼӼ�����
													 
void NdAlarmHandle(void)//Ũ�ȡ����������ϡ���ص�״̬���³���
{
	
		
}

