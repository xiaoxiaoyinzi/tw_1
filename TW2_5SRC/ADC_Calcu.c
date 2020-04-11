/*****************************************************************************
  *文件名: F051test/ADC_Calcu.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/06
  *描@@述：采用低功耗的方式采样AN4~AN7，并计算实际值,采集流量计的脉冲输出
********************************************************************************/
#include "main.h"

//定义私有define
#define ADC1_DR_Address                0x40012440
#define ADC_AI1 4
#define ADC_AI2 5
#define ADC_T 	6
#define ADC_BTY 7
#define Vref 2500  //ADC 参考电压
#define VoltageSensor 2500 
#define ResistorLoad 2000 
#define LineB 3.699;


volatile uint16_t ADC_AI1Value,ADC_AI2Value,ADC_TValue,ADC_BTYValue;//单次转换的值
volatile uint32_t SumAI1Value,SumAI2Value,SumTValue,SumBTYValue;//32次转换的值
volatile double AVAI1Value,AVAI2Value,AVTValue,AVBTYValue;//32次转换的平均值
double VoltageRs;//Rs上的分压，Rs节GND,RL接2.5V
double ResistorSensor;//传感器电阻Rs
const double LineK[25]={0,-2.06327,-2.05828,-2.06409,-2.06241,-2.05984,-2.08674,-2.07243,-2.05984,//0-8
                          -2.05984,-2.07441,-2.03392,-2.08396,-2.0817, -2.06476,-2.06881,-2.05984,//9-18
                          -2.05616,-2.06656,-2.0568, -2.06539,-2.06235,-2.06212,-2.06817,-2.06551};
uint8_t IndexSensor=14;
const uint16_t Resistor0[25]={0,715,787,866,953,1050,1150,1270,1400,1540,1690,1870,2050,//0-12,
                           2260,2490,2740,3010,3320,3650,4020,4420,4870,4360,5900,6490};//13-24		
const uint16_t Resistor1000[25]={0,1560,1720,1889,2080,2294,2487,2761,3058,3364,3672,4126,4438,//0-12,
                           4897,5429,5965,6575,7263,7953,8792,9635,10628,11699,12848,14147};//13-24		
double VaulueX;//Y=K*X+b中的X值，也就是lg(Rs/R0)的值
double VaulueY;//Y=K*X+b中的Y值，也就是lg(c)的值													 
double NdValueTemp1;//临时浓度的值
double NdValueTemp2;//临时浓度的值
uint32_t FlagBiaod10;//10%LEL标定标志，0x55表示已标定，存在相对FLASH地址0100	
uint32_t FlagBiaod50;//50%LEL标定标志，0x55表示已标定
uint8_t BiaodVar;//标定参数		
uint16_t Resistor0BD;//在10%LEL标定过的R0	
double K50;////在50%LEL标定过的K值
extern uint8_t EEArray[488];//对应488个模拟EE地址
uint16_t NdValue;//计算之后的气体浓度
uint8_t NdValueLEL;//0-100%LEL浓度

int16_t PT1,PT2;//单位KPa
int16_t PT1Old,PT2Old;//单位KPa
volatile double Tdouble;//温度实际值
volatile int8_t Tint;//温度实际值
volatile uint32_t FTgklj,FTgkss;//工况累计流量,工况瞬时流量
char VBtyAscii[4]={'3','.','6','7'};//电池电压ASCII 例如"3.67"
char FTbkljAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTgkljAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTbkssAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char FTgkssAscii[10]={'%','0','0','0','0','0','0','0','0','$'};
char PT1Ascii[6]={'%','0','0','0','0','$'};
char TTAscii[4]={'%','0','0','$'};

uint8_t FlagAlmH;//报警标志，0：正常，1：高报
uint8_t CntAlmH;//已报警次数
uint8_t FlagAlmL;//报警标志，0：正常，1：低报
uint8_t CntAlmL;//已报警次数
uint8_t FlagErr;//故障标志，0：正常，1：故障
uint8_t CntErr;//故障次数
uint8_t FlagWat;//水位标志，0：正常，1：水位
uint8_t FlagWatOld;//水位标志，0：正常，1：水位
uint8_t CntWat;//水位计数器
uint8_t FlagCat;//水位标志，0：正常，1：水位
uint8_t FlagCatOld;//水位标志，0：正常，1：水位
uint8_t CntCat;//水位计数器
uint8_t FlagBtyL;//欠压标志，0:正常,1:欠压
char TranArray[7]={'A','0','0','0','0','Z','\0'};//发送前装填的数组
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
	
	//GPIOA时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM3时钟使能
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//AD_BTY:PA5->AD_IN5,AD_CO:PA6->AD_IN6, AD_T:PA7->ADC_IN7
	//ADC Channel4~ADC Channel7 作为ADC使用 
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
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;//TIM3触发ADC
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
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//等待转换结束
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
	if(ADC_Chanel==ADC_AI1) GPIO_SetBits(GPIOA, GPIO_Pin_4);//开漏输出高电平，功耗最低
	else if(ADC_Chanel==ADC_AI2) GPIO_SetBits(GPIOA, GPIO_Pin_5);//开漏输出高电平，功耗最低
	else if(ADC_Chanel==ADC_T) GPIO_SetBits(GPIOA, GPIO_Pin_6);//开漏输出高电平，功耗最低
	else if(ADC_Chanel==ADC_BTY) GPIO_SetBits(GPIOA, GPIO_Pin_7);//开漏输出高电平，功耗最低
	*/
}


void ReadADCValue(void)//读取AI1\AI2\ADC_T\ADC_BTY的值
{
	uint16_t i;
	GPIO_InitTypeDef GPIO_InitStruct; //定义结构体
	
	VCCTempOn();//为LMT84供3.3V电
	DelayNms(100);//每个单位10ms
	//采集T
	ADC_TValue=0;SumTValue=0;
	for(i=0;i<32;i++){
		Init_ADC(ADC_T);Delay(2);//120us
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//等待转换结束
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
	//GPIO_ResetBits(GPIOB, GPIO_Pin_1);//关闭LMT84
	VCCTempOff();//关闭LMT84

	//采集AI1,TGS2611	
	if(WatL()){
		EN5VOn();//输出低电平，启动5V
		DelayNms(3000);//每个单位10ms，加热30秒
		ADC_AI1Value=0;SumAI1Value=0;
		for(i=0;i<32;i++){
			Init_ADC(ADC_AI1);Delay(2);//120us
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//等待转换结束
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
			ADC_AI1Value = ADC_GetConversionValue(ADC1);
			SumAI1Value += ADC_AI1Value;
			Delay(2);//120us
		}
		//SumAI1Value=SumAI1Value>>5;
		//AVAI1Value = (double)(SumAI1Value>>5);
		NdAlarmHandle((uint16_t)(SumAI1Value>>5));

		//采集AI2
		ADC_AI2Value=0;SumAI2Value=0;
		for(i=0;i<32;i++){
			Init_ADC(ADC_AI2);Delay(2);//120us
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//等待转换结束
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();}
			ADC_AI2Value = ADC_GetConversionValue(ADC1);
			SumAI2Value += ADC_AI2Value;	
			Delay(2);//120us
		}
		AVAI2Value = ((((double)(SumAI2Value>>5) )/4096)*Vref)*2;
		if(AVAI2Value<500){//0.5V对应0KPa
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
		EN5VOff();//高电平，关闭5V
	}
	
	//采集BTY	
	BTY_ENOn();//低电平，使能电池检测
	DelayNms(1);//每个单位10ms

	ADC_BTYValue=0;SumBTYValue=0;
	for(i=0;i<32;i++){
		Init_ADC(ADC_BTY);Delay(2);//120us
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){//等待转换结束
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
	//char VBtyAscii[4]={'3','.','6','7'};//电池电压ASCII 例如"3.67"
	VBtyAscii[0] = tbl_ascii[(uint8_t)(AVBTYValue/1000)];
	VBtyAscii[1] = '.';
	VBtyAscii[2] = tbl_ascii[(uint8_t)(((uint16_t)AVBTYValue%1000)/100)];
	VBtyAscii[3] = tbl_ascii[(uint8_t)(((uint16_t)AVBTYValue%100)/10)];
	GPIO_SetBits(GPIOB, GPIO_Pin_12);//高电平，禁止电池检测
	BTY_ENOff();//高电平，禁止电池检测
	
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//开漏输出高电平，功耗最低
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);//开漏输出高电平，功耗最低
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);//开漏输出高电平，功耗最低
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);//开漏输出高电平，功耗最低
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);//关闭会影响2个串口工作
	TIM_Cmd(TIM2, DISABLE);//TIM2 disable counter
	ADC_Cmd(ADC1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);//ADC1时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);//TIM3时钟使能
	Init_GPIO();
}

void CompPT(void)
{	
	float DeltaPT1;
	if(PT1<PT1Old){
		DeltaPT1=PT1Old-PT1;
		if((DeltaPT1/PT1Old)>0.5){//压力下降50%以上
			ComUDP(1);//与服务器UDP通讯
		}
	}
}

void NdAlarmHandle(uint16_t AD2611)//浓度、报警、故障判断
{
	uint8_t IndexNdLEL;
	//浓度计算、报警处理、状态更新
	ReadBiaoding();//读EE中标定参数
	VoltageRs=(AD2611*Vref)/4096;//Rs上的分压，Rs接GND,RL接2.5V
	ResistorSensor=VoltageRs*ResistorLoad/(VoltageSensor-VoltageRs);
	//if(ResistorSensor>=Resistor1000[IndexSensor])//理论计算小于1000ppm时，确保高灵敏度
	 //理论计算大于1000ppm时，采用lg(c)=k*lg(Rs/R0)+b公式计算浓度


	if(AD2611<10){// 20->0.16mv,传感器短路	
		NdValue=0 & 0xFFFF;//浓度为0，单位为ppm,0xFFFF用来表示单位，与0相与无实际意义			
		FlagErr=1;//电压小于0.16mV,传感器短路故障
		FlagAlmH=0;FlagAlmL=0;
	}else if(AD2611>4094 ){//判断断路故障
		NdValue=0 & 0xFFFF;//浓度为0，单位为ppm,0x3FFF用来表示单位，与0相与无实际意义
		FlagErr=1;//电压大于2.48V,传感器断路故障
		FlagAlmH=0;FlagAlmL=0;
	}else{ //无故障
		FlagErr=0;//无故障
		if(FlagBiaod10==1){//已经标定了10%LEL
			VaulueX=log10(ResistorSensor/Resistor0BD);
		}else{
			VaulueX=log10(ResistorSensor/Resistor0[IndexSensor]);
		}
		
		VaulueY=(-2.01)*VaulueX+3.699;
		NdValueTemp1=pow(10, VaulueY);
		
				
		if(FlagBiaod50==1){//50%LEL标定
			if(NdValueTemp1>5000){
				NdValueTemp2=K50*(NdValueTemp1-5000)+5000;
			}else{
				NdValueTemp2=NdValueTemp1;
			}
		}else{//未标定
			if(NdValueTemp1>5000){
				//NdValueTemp2=2*(NdValueTemp1-5000)+5000;
				//NdValueTemp2=1.1*(NdValueTemp1-5000)+5000;
				//NdValueTemp2=NdValueTemp1;
				NdValueTemp2=1.05*(NdValueTemp1-5000)+5000;
			}else{
				NdValueTemp2=NdValueTemp1;
			}		
		}
		if(NdValueTemp2>65000) NdValueTemp2 = 65000;//最大到130%LEL
		NdValue=(uint16_t)NdValueTemp2 & 0xFFFF;//bit15\bit14为00，单位为ppm
		IndexNdLEL=NdValue/250;//得到200个有效的索引值
		if(IndexNdLEL>202) IndexNdLEL=202;
		NdValueLEL=tbl_NdValue[IndexNdLEL];//利用有效的索引值查表
		//NdValueLEL=NdValue/500;
		if(NdValueLEL>100){
			NdValueLEL=100;
		}
		if((NdValueLEL>16)&&(NdValueLEL<24)) NdValueLEL=20;
		if((NdValueLEL>46)&&(NdValueLEL<54)) NdValueLEL=50;
		if(NdValueLEL>=AlmPointH){//50%LEL,高报
			FlagAlmH=1;FlagAlmL=1;
		}else if(NdValueLEL>=AlmPointL){//20%LEL,低报
			FlagAlmH=0;FlagAlmL=1;
		}else{
			FlagAlmH=0;FlagAlmL=0;//无报警
		}
	}	//计算浓度结束	
}

void NdBiaoding(uint8_t valuebd)//标定操作
{
	uint32_t Rs;
	uint32_t K50x1000;//50%LEL的K值的1000倍
	BiaodVar=valuebd;//全局变量是为了观察标定参数用
	ReadADCValue();//重新进行一次AD读取操作
	if(BiaodVar==10){//执行10%LEL标定
		EEArray[4]=1;//写EE标志
		Rs=(uint32_t)ResistorSensor;
		EEArray[8]=(uint8_t)(Rs&0x000000FF);
		EEArray[9]=(uint8_t)((Rs>>8)&0x000000FF);
		EEArray[10]=(uint8_t)((Rs>>16)&0x000000FF);
		EEArray[11]=(uint8_t)((Rs>>24)&0x000000FF);
		RefreshEE();//刷新EE
	}else if(BiaodVar==50){//执行50%LEL标定
		if(NdValueTemp1>5000){
			EEArray[12]=1;//写EE标志
			
			K50x1000=(uint32_t)((20000/(NdValueTemp1-5000))*1000);
			EEArray[16]=(uint8_t)(K50x1000&0x000000FF);
			EEArray[17]=(uint8_t)((K50x1000>>8)&0x000000FF);
			EEArray[18]=(uint8_t)((K50x1000>>16)&0x000000FF);
			EEArray[19]=(uint8_t)((K50x1000>>24)&0x000000FF);
			RefreshEE();//刷新EE
		}
	}
	//标定完成指示
	LEDRunOn();LEDSetOff(); DelayNms(100);
	LEDRunOff();LEDSetOn();DelayNms(50);
	LEDRunOn();LEDSetOn(); DelayNms(100);
	LEDRunOff();LEDSetOff();DelayNms(50);
	LEDRunOn();LEDSetOn(); DelayNms(100);
	LEDRunOff();LEDSetOff();DelayNms(1);
}

void ReadBiaoding(void)//读标定参数
{
	uint32_t K50INT32;
	FlagBiaod10=FlashEERead(4) & 0x000000FF;//读10%LEL标志
	
	EEArray[4]=FlagBiaod10;
	Resistor0BD=FlashEERead(8);//读10%LEL电阻值
	EEArray[8]=Resistor0BD & 0x000000FF;
	EEArray[9]=(Resistor0BD>>8) & 0x000000FF;
	EEArray[10]=(Resistor0BD>>16) & 0x000000FF;
	EEArray[11]=(Resistor0BD>>24) & 0x000000FF;
	if((Resistor0BD==0)||(Resistor0BD>20000))
		FlagBiaod10=0;//R0值超范围，清标定标志
	
	FlagBiaod50=FlashEERead(12) & 0x000000FF;//读50%LEL标志
	EEArray[12]=FlagBiaod50;
	K50INT32=FlashEERead(16);
	K50=((double)K50INT32)/1000;//读50%LEL标定K值
	EEArray[16]=K50INT32 & 0x000000FF;
	EEArray[17]=(K50INT32>>8) & 0x000000FF;
	EEArray[18]=(K50INT32>>16) & 0x000000FF;
	EEArray[19]=(K50INT32>>24) & 0x000000FF;
}

void AlmErr10min(void)//10分钟例行报警、故障检测处理程序
{
	ReadADCValue();
	//高报处理
	if(FlagAlmH==1){
		if(CntAlmH<NumAlmHMax){
			//1.装载报警数组
			TranArray[1]='2';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranAlmH);//2.发送报警数据			
			CntAlmH++;//3.CntAlm++
		}else{
			CntAlmH=NumAlmHMax;//超过预定发送次数，不再频繁发送
		}
	}else{
		if(CntAlmH==0){
			__nop();
		}else if((FlagAlmH!=1)&&(FlagErr!=1)){
			TranArray[1]='0';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranNorm);//2.发送报警数据	
			CntAlmH=0;
		}
	}
	//低报处理
	if((FlagAlmL==1)&&(FlagAlmH!=1)){
		if(CntAlmL<NumAlmLMax){
			//1.装载报警数组
			TranArray[1]='1';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranAlmL);//2.发送报警数据			
			CntAlmL++;//3.CntAlm++
		}else{
			CntAlmL=NumAlmLMax;//超过预定发送次数，不再频繁发送
		}
	}else{
		if(CntAlmL==0){
			__nop();
		}else if(FlagErr!=1){
			TranArray[1]='0';
			TranArray[2]=tbl_ascii[NdValueLEL/100];
			TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
			TranArray[4]=tbl_ascii[NdValueLEL%10];
			ComUDP(TranNorm);//2.发送报警数据	
			CntAlmL=0;
		}
	}
	//故障处理
	if(FlagErr==1){
		if(CntErr<NumErrMax){
			//1.装载报警数组
			TranArray[1]='3';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranAlmL);//2.发送报警数据			
			CntErr++;//3.CntAlm++
		}else{
			CntErr=NumErrMax;
		}
	}else{
		if(CntErr==0){//上周期没有传感器故障
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

void AlmErrWat(void)//水位触发报警、故障检测处理程序
{
	ReadADCValue();
	if(FlagAlmH==1){//1.高报
		TranArray[1]='2';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		CntAlmH = 1;//bing jia
		ComUDP(TranAlmH);		
	}else if(FlagAlmL==1){//2.低报
		TranArray[1]='1';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		CntAlmL = 1;//bing jia
		ComUDP(TranAlmL);		
	}else if(FlagErr==1){//3.故障
		TranArray[1]='3';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranErr);	
	}else{//4.正常
		TranArray[1]='0';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranNorm);
	}
}

void WatDecAlm(void)//水位报警处理程序，每1秒执行一次
{
	//1.水位传感器状态检测
	if(WatH()){
		CntWat++;
		if(CntWat>10){CntWat=10;FlagWat=1;}//10S,水位确认
		__nop(); //
	}else{
		if((CntWat>4)&&(CntWat<10)){//4-10S,水位触发检测
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(50);
			LEDRunOn();LEDSetOn(); DelayNms(100);
			LEDRunOff();LEDSetOff();DelayNms(1);
			AlmErrWat();//检测并上传数据
		}//触发操作结束
		CntWat=0;FlagWat=0;
		//FlagWatOld=0;
	}
	__nop();
	//2.水位逻辑处理
	if(FlagWat==1){//本周期有水位
		if(FlagWatOld==1){//上周期有水位
			__nop();
		}else{
			FlagWatOld=1;
			TranArray[1]='4';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='0';
			ComUDP(TranWat);//上传水位报警
		}
		EN5VOff();//高电平，关闭5V
	}else{//本周期无水位
		if(FlagWatOld==1){//上周期有水位
			FlagWatOld=0;		
			TranArray[1]='4';
			TranArray[2]='0';
			TranArray[3]='0';
			TranArray[4]='5';
			ComUDP(TranWat);//上传水位恢复
			AlmErrWat();
		}else{
			__nop();
		}
	}
}
void CatDecAlm(void)//CCCCC
{
	//1.拆卸
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
		}//触发操作结束
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
		EN5VOff();//高电平，关闭5V
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
void Hart6Hour(void)//6小时心跳数据上传程序
{
	if(FlagAlmH==1){//1.高报
		TranArray[1]='2';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranAlmH);		
	}else if(FlagAlmL==1){//2.低报
		TranArray[1]='1';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranAlmL);		
	}else if(FlagErr==1){//3.故障
		TranArray[1]='3';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranErr);	
	}else if(FlagWat==1){//4.水位
		TranArray[1]='4';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranWat);	
	}else if(FlagCat==1){//4.拆卸
		TranArray[1]='8';
		TranArray[2]='0';
		TranArray[3]='0';
		TranArray[4]='0';
		ComUDP(TranCat);	
	}else if(FlagBtyL==1){//5.欠压
		TranArray[1]='5';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranErr);	
	}else{//6.正常
		TranArray[1]='0';
		TranArray[2]=tbl_ascii[NdValueLEL/100];
		TranArray[3]=tbl_ascii[(NdValueLEL%100)/10];
		TranArray[4]=tbl_ascii[NdValueLEL%10];
		ComUDP(TranNorm);
	}
}

