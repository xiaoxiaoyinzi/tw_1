/*****************************************************************************
  *�ļ���: F051test/ADC_Calcu.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/06
  *��@@�������õ͹��ĵķ�ʽ����AN4~AN7��������ʵ��ֵ,�ɼ������Ƶ��������
********************************************************************************/
#include "main.h"

//����˽��define
#define ADC1_DR_Address                0x40012440
#define ADC_AI1 4
#define ADC_AI2 5
#define ADC_T 	6
#define ADC_BTY 7
#define Vref 2500  //ADC �ο���ѹ
#define VoltageSensor 2500 
#define ResistorLoad 2000 
#define LineB 3.699;


volatile uint16_t ADC_AI1Value,ADC_AI2Value,ADC_TValue,ADC_BTYValue;//����ת����ֵ
volatile uint32_t SumAI1Value,SumAI2Value,SumTValue,SumBTYValue;//32��ת����ֵ
volatile double AVAI1Value,AVAI2Value,AVTValue,AVBTYValue;//32��ת����ƽ��ֵ
double VoltageRs;//Rs�ϵķ�ѹ��Rs��GND,RL��2.5V
double ResistorSensor;//����������Rs
const double LineK[25]={0,-2.06327,-2.05828,-2.06409,-2.06241,-2.05984,-2.08674,-2.07243,-2.05984,//0-8
                          -2.05984,-2.07441,-2.03392,-2.08396,-2.0817, -2.06476,-2.06881,-2.05984,//9-18
                          -2.05616,-2.06656,-2.0568, -2.06539,-2.06235,-2.06212,-2.06817,-2.06551};
uint8_t IndexSensor=14;
const uint16_t Resistor0[25]={0,715,787,866,953,1050,1150,1270,1400,1540,1690,1870,2050,//0-12,
                           2260,2490,2740,3010,3320,3650,4020,4420,4870,4360,5900,6490};//13-24		
const uint16_t Resistor1000[25]={0,1560,1720,1889,2080,2294,2487,2761,3058,3364,3672,4126,4438,//0-12,
                           4897,5429,5965,6575,7263,7953,8792,9635,10628,11699,12848,14147};//13-24		
double VaulueX;//Y=K*X+b�е�Xֵ��Ҳ����lg(Rs/R0)��ֵ
double VaulueY;//Y=K*X+b�е�Yֵ��Ҳ����lg(c)��ֵ													 
double NdValueTemp1;//��ʱŨ�ȵ�ֵ
double NdValueTemp2;//��ʱŨ�ȵ�ֵ
uint32_t FlagBiaod10;//10%LEL�궨��־��0x55��ʾ�ѱ궨���������FLASH��ַ0100	
uint32_t FlagBiaod50;//50%LEL�궨��־��0x55��ʾ�ѱ궨
uint8_t BiaodVar;//�궨����		
uint16_t Resistor0BD;//��10%LEL�궨����R0	
double K50;////��50%LEL�궨����Kֵ
extern uint8_t EEArray[488];//��Ӧ488��ģ��EE��ַ
uint16_t NdValue;//����֮�������Ũ��
uint8_t NdValueLEL;//0-100%LELŨ��

int16_t PT1,PT2;//��λKPa
int16_t PT1Old,PT2Old;//��λKPa
volatile double Tdouble;//�¶�ʵ��ֵ
volatile int8_t Tint;//�¶�ʵ��ֵ
volatile uint32_t FTgklj,FTgkss;//�����ۼ�����,����˲ʱ����
char VBtyAscii[4]={'3','.','6','7'};//��ص�ѹASCII ����"3.67"
char FTbkljAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTgkljAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTbkssAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTgkssAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char PT1Ascii[6]={'%','0','0','0','0','$'};
char TTAscii[4]={'%','0','0','$'};

uint8_t FlagAlmH;//������־��0��������1���߱�
uint8_t CntAlmH;//�ѱ�������
uint8_t FlagAlmL;//������־��0��������1���ͱ�
uint8_t CntAlmL;//�ѱ�������
uint8_t FlagErr;//���ϱ�־��0��������1������
uint8_t CntErr;//���ϴ���
uint8_t FlagWat;//ˮλ��־��0��������1��ˮλ
uint8_t FlagWatOld;//ˮλ��־��0��������1��ˮλ
uint8_t CntWat;//ˮλ������
uint8_t FlagCat;//ˮλ��־��0��������1��ˮλ
uint8_t FlagCatOld;//ˮλ��־��0��������1��ˮλ
uint8_t CntCat;//ˮλ������
uint8_t FlagBtyL;//Ƿѹ��־��0:����,1:Ƿѹ
char TranArray[7]={'A','0','0','0','0','Z','\0'};//����ǰװ�������
extern const int8_t tbl_ascii[];

uint8_t tbl_NdValue[203]={0,0,1,2,3,4,5,6,7,7,8,8,8,8,9,9,9,9,9,9,//0-9
	10,10,11,11,12,12,13,13,14,15,16,17,18,18,18,19,19,19,20,20,//10-19
	20,20,20,20,20,20,21,21,21,22,22,22,23,23,24,25,26,27,28,29,//20-29
	30,30,31,31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,//30-39
	40,40,41,41,42,42,43,43,44,45,46,47,48,48,48,49,49,49,50,50,//40-49
	50,50,50,50,51,51,51,51,51,52,52,52,53,53,54,55,56,57,58,59,//50-59
	60,60,61,61,62,62,63,63,64,64,65,65,66,66,67,67,68,68,69,69,//60-69
	70,70,71,71,72,72,73,73,74,74,75,75,76,76,77,77,78,78,79,79,//70-79
	80,80,81,81,82,82,83,83,84,84,85,85,86,86,87,87,88,88,89,89,//80-89
	90,90,91,91,92,92,93,93,94,94,95,95,96,96,97,97,98,98,99,99,//90-99
	100,100,100//100
};

void Init_ADC(uint8_t ADC_Chanel)
{
//	uint8_t i;
	GPIO_InitTypeDef         GPIO_InitStruct;
	ADC_InitTypeDef          ADC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure; 
	
	//GPIOAʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1ʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM3ʱ��ʹ��
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//AD_BTY:PA5->AD_IN5,AD_CO:PA6->AD_IN6, AD_T:PA7->ADC_IN7
	//ADC Channel4~ADC Channel7 ��ΪADCʹ�� 
	if(ADC_Chanel==ADC_AI1) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//ADC_AI1/ADC_AI2/ADC_T/ADC_BTY
	else if(ADC_Chanel==ADC_AI2) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	else if(ADC_Chanel==ADC_T) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	else if(ADC_Chanel==ADC_BTY) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct); 
	//TIM3 
  TIM_DeInit(TIM2); 
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);  
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 0xFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  // TIM3 TRGO selection 
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	
	// ADC1 Configuration 
  ADC_DeInit(ADC1);//ADCs DeInit  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12 bits
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//Configure the ADC1 in continous mode
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;    
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;//TIM3����ADC
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
	
	if(ADC_Chanel==ADC_AI1) ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_13_5Cycles);//7.5 13.5 28.5 55.5 71.5 239.5
	else if(ADC_Chanel==ADC_AI2) ADC_ChannelConfig(ADC1, ADC_Channel_5 , ADC_SampleTime_13_5Cycles);
	else if(ADC_Chanel==ADC_T) ADC_ChannelConfig(ADC1, ADC_Channel_6 , ADC_SampleTime_13_5Cycles);
	else if(ADC_Chanel==ADC_BTY) ADC_ChannelConfig(ADC1, ADC_Channel_7 , ADC_SampleTime_13_5Cycles);
	
	ADC_GetCalibrationFactor(ADC1);//ADC Calibration 
	ADC_Cmd(ADC1, ENABLE);//Enable ADCperipheral[PerIdx]     
  //while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));// Wait the ADCEN flag
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)){// Wait the ADRDY flag 
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
  ADC_WaitModeCmd(ADC1, ENABLE);//Enable the auto delay feature
  ADC_AutoPowerOffCmd(ADC1, ENABLE);//Enable the Auto power off mode 

  TIM_Cmd(TIM2, ENABLE);//TIM2 enable counter
  ADC_StartOfConversion(ADC1);//ADC1 regular Software Start Conv
	/*
	for(i=0;i<32;i++){
		TIM_Cmd(TIM3, ENABLE);//TIM2 enable counter
		ADC_StartOfConversion(ADC1);//ADC1 regular Software Start Conv
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//�ȴ�ת������
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
		ADC_AI2Value = ADC_GetConversionValue(ADC1);
		SumAI2Value += ADC_AI2Value;	
		TIM_Cmd(TIM3, DISABLE);//TIM2 enable counter
	}  
	AVAI1Value  = SumAI2Value / 32;;
	i=0;
	
	if(ADC_Chanel==ADC_AI1) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//ADC_AI1/ADC_AI2/ADC_T/ADC_BTY
	else if(ADC_Chanel==ADC_AI2) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	else if(ADC_Chanel==ADC_T) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	else if(ADC_Chanel==ADC_BTY) GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOB, &GPIO_InitStruct); //
	if(ADC_Chanel==ADC_AI1) GPIO_SetBits(GPIOA, GPIO_Pin_4);//��©����ߵ�ƽ���������
	else if(ADC_Chanel==ADC_AI2) GPIO_SetBits(GPIOA, GPIO_Pin_5);//��©����ߵ�ƽ���������
	else if(ADC_Chanel==ADC_T) GPIO_SetBits(GPIOA, GPIO_Pin_6);//��©����ߵ�ƽ���������
	else if(ADC_Chanel==ADC_BTY) GPIO_SetBits(GPIOA, GPIO_Pin_7);//��©����ߵ�ƽ���������
	*/
}


void ReadADCValue(void)//��ȡAI1\AI2\ADC_T\ADC_BTY��ֵ
{
	uint16_t i;
	GPIO_InitTypeDef GPIO_InitStruct; //����ṹ��
	
	VCCTempOn();//ΪLMT84��3.3V��
	DelayNms(100);//ÿ����λ10ms
	//�ɼ�T
	ADC_TValue=0;SumTValue=0;
	for(i=0;i<32;i++){
		Init_ADC(ADC_T);Delay(2);//120us
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//�ȴ�ת������
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
		ADC_TValue = ADC_GetConversionValue(ADC1);
		SumTValue += ADC_TValue;	
		Delay(2);//120us
	}
//bing	AVTValue = (((double)(SumTValue>>5))/4096)*Vref;
	AVTValue = (((double)(SumTValue>>5))*25)/40.96;
	Tdouble = (AVTValue-925)*(-0.2174)+20;
	Tint =(int8_t)Tdouble;//char TTAscii[4]={'%','0','0','$'};
	TTAscii[2]=tbl_ascii[Tint&0x0F];
	TTAscii[1]=tbl_ascii[(Tint>>4)&0x0F];
	//GPIO_ResetBits(GPIOB, GPIO_Pin_1);//�ر�LMT84
	VCCTempOff();//�ر�LMT84

	//�ɼ�AI1,TGS2611	
	if(WatL()){
		EN5VOn();//����͵�ƽ������5V
		DelayNms(3000);//ÿ����λ10ms������30��
		ADC_AI1Value=0;SumAI1Value=0;
		for(i=0;i<32;i++){
			Init_ADC(ADC_AI1);Delay(2);//120us
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//�ȴ�ת������
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
			ADC_AI1Value = ADC_GetConversionValue(ADC1);
			SumAI1Value += ADC_AI1Value;
			Delay(2);//120us
		}
		//SumAI1Value=SumAI1Value>>5;
		//AVAI1Value = (double)(SumAI1Value>>5);
		NdAlarmHandle((uint16_t)(SumAI1Value>>5));

		//�ɼ�AI2
		ADC_AI2Value=0;SumAI2Value=0;
		for(i=0;i<32;i++){
			Init_ADC(ADC_AI2);Delay(2);//120us
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//�ȴ�ת������
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
			ADC_AI2Value = ADC_GetConversionValue(ADC1);
			SumAI2Value += ADC_AI2Value;	
			Delay(2);//120us
		}
		AVAI2Value = ((((double)(SumAI2Value>>5) )/4096)*Vref)*2;
		if(AVAI2Value<500){//0.5V��Ӧ0KPa
			PT1=0;
		}else if(AVAI2Value>4500){
			PT1=1000;
		}else{
			PT1=(int16_t)(0.25*(AVAI2Value-500));
		} 
		//char PT1Ascii[6]={'%','0','0','0','0','$'};
		PT1Ascii[1]=tbl_ascii[(uint8_t)((PT1>>12)&0x000F)];
		PT1Ascii[2]=tbl_ascii[(uint8_t)((PT1>>8)&0x000F)];
		PT1Ascii[3]=tbl_ascii[(uint8_t)((PT1>>4)&0x000F)];
		PT1Ascii[4]=tbl_ascii[(uint8_t)(PT1&0x000F)];
		EN5VOff();//�ߵ�ƽ���ر�5V
	}
	
	//�ɼ�BTY	
	BTY_ENOn();//�͵�ƽ��ʹ�ܵ�ؼ��
	DelayNms(1);//ÿ����λ10ms

	ADC_BTYValue=0;SumBTYValue=0;
	for(i=0;i<32;i++){
		Init_ADC(ADC_BTY);Delay(2);//120us
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//�ȴ�ת������
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
		ADC_BTYValue = ADC_GetConversionValue(ADC1);
		SumBTYValue += ADC_BTYValue;
		Delay(2);//120us
	}
	AVBTYValue = (((double)(SumBTYValue>>5))/4096*Vref)*2;
	if(AVBTYValue < 3000)
		FlagBtyL = 1;
	else
		FlagBtyL = 0;
	//char VBtyAscii[4]={'3','.','6','7'};//��ص�ѹASCII ����"3.67"
	VBtyAscii[0] = tbl_ascii[(uint8_t)(AVBTYValue/1000)];
	VBtyAscii[1] = '.';
	VBtyAscii[2] = tbl_ascii[(uint8_t)(((uint16_t)AVBTYValue%1000)/100)];
	VBtyAscii[3] = tbl_ascii[(uint8_t)(((uint16_t)AVBTYValue%100)/10)];
	GPIO_SetBits(GPIOB, GPIO_Pin_12);//�ߵ�ƽ����ֹ��ؼ��
	BTY_ENOff();//�ߵ�ƽ����ֹ��ؼ��
	
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//��©����ߵ�ƽ���������
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);//��©����ߵ�ƽ���������
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);//��©����ߵ�ƽ���������
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);//��©����ߵ�ƽ���������
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);//�رջ�Ӱ��2�����ڹ���
	TIM_Cmd(TIM2, DISABLE);//TIM2 disable counter
	ADC_Cmd(ADC1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);//ADC1ʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);//TIM3ʱ��ʹ��
	Init_GPIO();
}

void CompPT(void)
{	
	float DeltaPT1;
	if(PT1<PT1Old){
		DeltaPT1=PT1Old-PT1;
		if((DeltaPT1/PT1Old)>0.5){//ѹ���½�50%����
			ComUDP(1);//�������UDPͨѶ
		}
	}
}

void NdAlarmHandle(uint16_t AD2611)//Ũ�ȡ������������ж�
{
	uint8_t IndexNdLEL;
	//Ũ�ȼ��㡢��������״̬����
	ReadBiaoding();//��EE�б궨����
	VoltageRs=(AD2611*Vref)/4096;//Rs�ϵķ�ѹ��Rs��GND,RL��2.5V
	ResistorSensor=VoltageRs*ResistorLoad/(VoltageSensor-VoltageRs);
	//if(ResistorSensor>=Resistor1000[IndexSensor])//���ۼ���С��1000ppmʱ��ȷ����������
	 //���ۼ������1000ppmʱ������lg(c)=k*lg(Rs/R0)+b��ʽ����Ũ��


	if(AD2611<10){// 20->0.16mv,��������·	
		NdValue=0 & 0xFFFF;//Ũ��Ϊ0����λΪppm,0xFFFF������ʾ��λ����0������ʵ������			
		FlagErr=1;//��ѹС��0.16mV,��������·����
		FlagAlmH=0;FlagAlmL=0;
	}else if(AD2611>4094 ){//�ж϶�·����
		NdValue=0 & 0xFFFF;//Ũ��Ϊ0����λΪppm,0x3FFF������ʾ��λ����0������ʵ������
		FlagErr=1;//��ѹ����2.48V,��������·����
		FlagAlmH=0;FlagAlmL=0;
	}else{ //�޹���
		FlagErr=0;//�޹���
		if(FlagBiaod10==1){//�Ѿ��궨��10%LEL
			VaulueX=log10(ResistorSensor/Resistor0BD);
		}else{
			VaulueX=log10(ResistorSensor/Resistor0[IndexSensor]);
		}
		
		VaulueY=(-2.01)*VaulueX+3.699;
		NdValueTemp1=pow(10, VaulueY);
		
				
		if(FlagBiaod50==1){//50%LEL�궨
			if(NdValueTemp1>5000){
				NdValueTemp2=K50*(NdValueTemp1-5000)+5000;
			}else{
				NdValueTemp2=NdValueTemp1;
			}
		}else{//δ�궨
			if(NdValueTemp1>5000){
				//NdValueTemp2=2*(NdValueTemp1-5000)+5000;
				//NdValueTemp2=1.1*(NdValueTemp1-5000)+5000;
				//NdValueTemp2=NdValueTemp1;
				NdValueTemp2=1.05*(NdValueTemp1-5000)+5000;
			}else{
				NdValueTemp2=NdValueTemp1;
			}		
		}
		if(NdValueTemp2>65000) NdValueTemp2 = 65000;//���130%LEL
		NdValue=(uint16_t)NdValueTemp2 & 0xFFFF;//bit15\bit14Ϊ00����λΪppm
		IndexNdLEL=NdValue/250;//�õ�200����Ч������ֵ
		if(IndexNdLEL>202) IndexNdLEL=202;
		NdValueLEL=tbl_NdValue[IndexNdLEL];//������Ч������ֵ���
		//NdValueLEL=NdValue/500;
		if(NdValueLEL>100){
			NdValueLEL=100;
		}
		if((NdValueLEL>16)&&(NdValueLEL<24)) NdValueLEL=20;
		if((NdValueLEL>46)&&(NdValueLEL<54)) NdValueLEL=50;
		if(NdValueLEL>=AlmPointH){//50%LEL,�߱�
			FlagAlmH=1;FlagAlmL=1;
		}else if(NdValueLEL>=AlmPointL){//20%LEL,�ͱ�
			FlagAlmH=0;FlagAlmL=1;
		}else{
			FlagAlmH=0;FlagAlmL=0;//�ޱ���
		}
	}	//����Ũ�Ƚ���	
}

void NdBiaoding(uint8_t valuebd)//�궨����
{
	uint32_t Rs;
	uint32_t K50x1000;//50%LEL��Kֵ��1000��
	BiaodVar=valuebd;//ȫ�ֱ�����Ϊ�˹۲�궨������
	ReadADCValue();//���½���һ��AD��ȡ����
	if(BiaodVar==10){//ִ��10%LEL�궨
		EEArray[4]=1;//дEE��־
		Rs=(uint32_t)ResistorSensor;
		EEArray[8]=(uint8_t)(Rs&0x000000FF);
		EEArray[9]=(uint8_t)((Rs>>8)&0x000000FF);
		EEArray[10]=(uint8_t)((Rs>>16)&0x000000FF);
		EEArray[11]=(uint8_t)((Rs>>24)&0x000000FF);
		RefreshEE();//ˢ��EE
	}else if(BiaodVar==50){//ִ��50%LEL�궨
		if(NdValueTemp1>5000){
			EEArray[12]=1;//дEE��־
			
			K50x1000=(uint32_t)((20000/(NdValueTemp1-5000))*1000);
			EEArray[16]=(uint8_t)(K50x1000&0x000000FF);
			EEArray[17]=(uint8_t)((K50x1000>>8)&0x000000FF);
			EEArray[18]=(uint8_t)((K50x1000>>16)&0x000000FF);
			EEArray[19]=(uint8_t)((K50x1000>>24)&0x000000FF);
			RefreshEE();//ˢ��EE
		}
	}
	//�궨���ָʾ
	LEDRunOn();LEDSetOff(); DelayNms(100);
	LEDRunOff();LEDSetOn();DelayNms(50);
	LEDRunOn();LEDSetOn(); DelayNms(100);
	LEDRunOff();LEDSetOff();DelayNms(50);
	LEDRunOn();LEDSetOn(); DelayNms(100);
	LEDRunOff();LEDSetOff();DelayNms(1);
}

void ReadBiaoding(void)//���궨����
{
	uint32_t K50INT32;
	FlagBiaod10=FlashEERead(4) & 0x000000FF;//��10%LEL��־
	
	EEArray[4]=FlagBiaod10;
	Resistor0BD=FlashEERead(8);//��10%LEL����ֵ
	EEArray[8]=Resistor0BD & 0x000000FF;
	EEArray[9]=(Resistor0BD>>8) & 0x000000FF;
	EEArray[10]=(Resistor0BD>>16) & 0x000000FF;
	EEArray[11]=(Resistor0BD>>24) & 0x000000FF;
	if((Resistor0BD==0)||(Resistor0BD>20000))
		FlagBiaod10=0;//R0ֵ����Χ����궨��־
	
	FlagBiaod50=FlashEERead(12) & 0x000000FF;//��50%LEL��־
	EEArray[12]=FlagBiaod50;
	K50INT32=FlashEERead(16);
	K50=((double)K50INT32)/1000;//��50%LEL�궨Kֵ
	EEArray[16]=K50INT32 & 0x000000FF;
	EEArray[17]=(K50INT32>>8) & 0x000000FF;
	EEArray[18]=(K50INT32>>16) & 0x000000FF;
	EEArray[19]=(K50INT32>>24) & 0x000000FF;
}

void AlmErr10min(void)//10�������б��������ϼ�⴦�����
{
	ReadADCValue();
	//�߱�����
	if(FlagAlmH==1){
		if(CntAlmH<NumAlmHMax){
			//1.װ�ر�������
			TranArray[1]='2';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranAlmH);//2.���ͱ�������			
			CntAlmH++;//3.CntAlm++
		}else{
			CntAlmH=NumAlmHMax;//����Ԥ�����ʹ���������Ƶ������
		}
	}else{
		if(CntAlmH==0){
			__nop();
		}else if((FlagAlmH!=1)&&(FlagErr!=1)){
			TranArray[1]='0';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranNorm);//2.���ͱ�������	
			CntAlmH=0;
		}
	}
	//�ͱ�����
	if((FlagAlmL==1)&&(FlagAlmH!=1)){
		if(CntAlmL<NumAlmLMax){
			//1.װ�ر�������
			TranArray[1]='1';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranAlmL);//2.���ͱ�������			
			CntAlmL++;//3.CntAlm++
		}else{
			CntAlmL=NumAlmLMax;//����Ԥ�����ʹ���������Ƶ������
		}
	}else{
		if(CntAlmL==0){
			__nop();
		}else if(FlagErr!=1){
			TranArray[1]='0';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranNorm);//2.���ͱ�������	
			CntAlmL=0;
		}
	}
	//���ϴ���
	if(FlagErr==1){
		if(CntErr<NumErrMax){
			//1.װ�ر�������
			TranArray[1]='3';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranAlmL);//2.���ͱ�������			
			CntErr++;//3.CntAlm++
		}else{
			CntErr=NumErrMax;
		}
	}else{
		if(CntErr==0){//������û�д���������
			__nop();
		}else if((FlagAlmL!=1)&&(FlagAlmH!=1)){
			TranArray[1]='0';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranNorm);
			CntErr=0;
		}
	}
}

void AlmErrWat(void)//ˮλ�������������ϼ�⴦�����
{
	ReadADCValue();
	if(FlagAlmH==1){//1.�߱�
		TranArray[1]='2';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		CntAlmH = 1;//bing jia
		ComUDP(TranAlmH);		
	}else if(FlagAlmL==1){//2.�ͱ�
		TranArray[1]='1';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		CntAlmL = 1;//bing jia
		ComUDP(TranAlmL);		
	}else if(FlagErr==1){//3.����
		TranArray[1]='3';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranErr);	
	}else{//4.����
		TranArray[1]='0';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranNorm);
	}
}

void WatDecAlm(void)//ˮλ�����������ÿ1��ִ��һ��
{
	//1.ˮλ������״̬���
	if(WatH()){
		CntWat++;
		if(CntWat>10){CntWat=10;FlagWat=1;}//10S,ˮλȷ��
		__nop(); //
	}else{
		if((CntWat>4)&&(CntWat<10)){//4-10S,ˮλ�������
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(50);
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(1);
			AlmErrWat();//��Ⲣ�ϴ�����
		}//������������
		CntWat=0;FlagWat=0;
		//FlagWatOld=0;
	}
	__nop();
	//2.ˮλ�߼�����
	if(FlagWat==1){//��������ˮλ
		if(FlagWatOld==1){//��������ˮλ
			__nop();
		}else{
			FlagWatOld=1;
			TranArray[1]='4';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranWat);//�ϴ�ˮλ����
		}
		EN5VOff();//�ߵ�ƽ���ر�5V
	}else{//��������ˮλ
		if(FlagWatOld==1){//��������ˮλ
			FlagWatOld=0;		
			TranArray[1]='4';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='5';
			ComUDP(TranWat);//�ϴ�ˮλ�ָ�
			AlmErrWat();
		}else{
			__nop();
		}
	}
}
void CatDecAlm(void)//CCCCC
{
	//1.��ж
	if(CatH()){
		CntCat++;
		if(CntCat>10){CntCat=10;FlagCat=1;}
		__nop(); //
	}else{
		if((CntCat>4)&&(CntCat<10)){
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(50);
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(1);
			AlmErrWat();
		}//������������
		CntCat=0;FlagCat=0;
		
	}
	__nop();
	
	if(FlagCat==1){
		if(FlagCatOld==1){
			__nop();
		}else{
			FlagCatOld=1;
			TranArray[1]='8';
		//	TranArray[1]='8';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranCat);
		}
		EN5VOff();//�ߵ�ƽ���ر�5V
	}else{
		if(FlagCatOld==1){
			FlagCatOld=0;		
			TranArray[1]='0';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranCat);
			AlmErrWat();
		}else{
			__nop();
		}
	}
}
void Hart6Hour(void)//6Сʱ���������ϴ�����
{
	if(FlagAlmH==1){//1.�߱�
		TranArray[1]='2';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranAlmH);		
	}else if(FlagAlmL==1){//2.�ͱ�
		TranArray[1]='1';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranAlmL);		
	}else if(FlagErr==1){//3.����
		TranArray[1]='3';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranErr);	
	}else if(FlagWat==1){//4.ˮλ
		TranArray[1]='4';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranWat);	
	}else if(FlagCat==1){//4.��ж
		TranArray[1]='8';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranCat);	
	}else if(FlagBtyL==1){//5.Ƿѹ
		TranArray[1]='5';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranErr);	
	}else{//6.����
		TranArray[1]='0';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranNorm);
	}
}

