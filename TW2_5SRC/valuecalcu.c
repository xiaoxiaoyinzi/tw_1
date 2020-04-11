#include "main.h"
#include "math.h"

//#define VoltageRef 3300 //参考电压为3.3V
#define VoltageRef 2500 //参考电压为2.5V
#define VoltageSensor 2500 //传感器信号部分供电电压2.5V
#define ResistorLoad 3300 //负载电阻3300欧姆，负载电阻接VoltageSensor
#define LineB 3.699;//公式lg(c)=k*lg(Rs/R0)+b中的b值

#define BAT_CHG_ON()       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 0)//正在充电
#define BAT_CHG_OFF()        (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 1)//未充电
#define BAT_PG_ON()       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 0)//BQ24041输入电压正常
#define BAT_PG_OFF()        (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 1)//BQ24041输入电压异常

uint16_t DeltaAVCOValue;//当前周期的AVCOValue与前一周期的差值
uint16_t DeltaAVCOValuePrev;//前一周期的AVCOValue与前二周期的差值
double VoltageBty;//换算成电池电压值
double VoltageRs;//Rs上的分压，Rs节GND,RL接2.5V
double ResistorSensor;//传感器电阻Rs
const double LineK[25]={0,-2.06327,-2.05828,-2.06409,-2.06241,-2.05984,-2.08674,-2.07243,-2.05984,//0-8
                          -2.05984,-2.07441,-2.03392,-2.08396,-2.0817, -2.06476,-2.06881,-2.05984,//9-18
                          -2.05616,-2.06656,-2.0568, -2.06539,-2.06235,-2.06212,-2.06817,-2.06551};//公式lg(c)=k*lg(Rs/R0)+b中的k值;
uint8_t IndexSensor=13;//斜率值和传感器R0值的索引号，默认为#14传感器，标定时修改
const uint16_t Resistor0[25]={0,715,787,866,953,1050,1150,1270,1400,1540,1690,1870,2050,//0-12,//传感器在5000ppm甲烷中的电阻值
                           2260,2490,2740,3010,3320,3650,4020,4420,4870,4360,5900,6490};//13-24		
const uint16_t Resistor1000[25]={0,1560,1720,1889,2080,2294,2487,2761,3058,3364,3672,4126,4438,//0-12,//传感器在1000ppm甲烷中的电阻值
                           4897,5429,5965,6575,7263,7953,8792,9635,10628,11699,12848,14147};//13-24		
double VaulueX;//Y=K*X+b中的X值，也就是lg(Rs/R0)的值
double VaulueY;//Y=K*X+b中的Y值，也就是lg(c)的值													 
double COValueTemp1;//临时浓度的值
float NdZero;//零点AD值，采用动态零点
uint8_t CntZero=0;//调零计数器，第一次进入NdAlarmHandle()开始计算次数，在第6次调零
uint8_t FlagZero=0xAA;//调零标志，未调零
													 											 
volatile uint16_t ADBtyValue,ADCOValue,ADTempValue;//单次转换的值
volatile uint32_t SumBtyValue,SumCOValue,SumTempValue;//64次转换的值
volatile uint16_t AVBtyValue,AVCOValue,AVTempValue;//64次转换的平均值
uint16_t COValue;//计算之后的气体浓度
uint8_t BtyValue;//计算之后的电池电压值
uint8_t SelfStat=0;//设备自身状态：bit0-报警,bit1-传故,bit2-欠压,bit3-充电,bit4-冲完
uint8_t CntADSum=0;//ad值累加计数器
													 
void NdAlarmHandle(void)//浓度、报警、故障、电池等状态更新程序
{
	
		
}

