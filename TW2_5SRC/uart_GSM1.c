/*****************************************************************************
  *文件名: F051test/uart_GSM.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/06
  *描@@述：通过uart2与GPRS模块通讯
********************************************************************************/
#include "main.h"
#include "string.h"
#include "stm32f0xx_it.h" //中断头文件在stm32f0xx_conf.h中未包含

//定义私有define

//GSM Const
char AT2ORDER[] = {'+','+','+',0x0D,'\0'};//数据模式转换成命令模式，change data mode to order mode
char AT2DATA[] = {'A','T','O',0x0D,'\0'};//命令模式转换成数据模式，change order mode to data mode
//char AT2DATA[] = {'A','T','O','\0'};//命令模式转换成数据模式，change order mode to data mode
const int8_t ATsimstu[] = {'A','T','*','T','S','I','M','I','N','S','?','\0'};//查SIM卡状态，ask the simcard status
const int8_t ATcops1[] = {'A','T','+','C','O','P','S','?','\0'};//查网络状态，ask the net status
//const uint8_t ATcops2[] = {'A','T','+','C','O','P','S','=','4',',','0',',','"',
//					'C','h','i','n','a',' ','M','o','b','i','l','e','"'};//select net 

char AT[]={'A','T',0X0D,'\0'};              //字符串的结尾为 '/0'
char ATE0[]={'A','T','E','0',0X0D,'\0'};    //关闭回显
char ATCPIN[]={'A','T','+','C','P','I','N','?',0X0D,'\0'};    //查看SIM卡状态
char ATCREG[]={'A','T','+','C','R','E','G','?',0X0D,'\0'};    //查看GSM网络注册状态
char ATCSQ[]={'A','T','+','C','S','Q',0X0D,'\0'};    //查看信号强度
char ATCGATT[]={'A','T','+','C','G','A','T','T','?',0X0D,'\0'};//查询GPRS附着状态
char ATCGATT1[]={'A','T','+','C','G','A','T','T','=','1',0X0D,'\0'};//强制GPRS附着
char ZPNum[]={'A','T','+','Z','P','N','U','M','=','"','C','M','N','E','T','"',
				      ',','"','U','S','E','R','"',',','"','P','W','D','"',0x0D,'\0'};
char ZPPPOpen[]={'A','T','+','Z','P','P','P','O','P','E','N',0x0D,'\0'};
char SETUPU[]={'A','T','+','Z','I','P','S','E','T','U','P','U','=','1',',','1','2','4','.',
				       '1','2','8','.','2','0','0','.','1','9','2',',','5','0','0','8',0x0D,'\0'};
char SETUP[]={'A','T','+','Z','I','P','S','E','T','U','P','=','1',',','1','2','4','.',
				       '1','2','8','.','2','0','0','.','1','9','2',',','5','0','0','8',0x0D,'\0'};
char TRANSFER[]={'A','T','+','Z','T','R','A','N','S','F','E','R','=','1',',','1',',',
				         '1','0','0','0',',','1','0','0','0',0x0D,'\0'};
char TRANSFER_TCP[]={'A','T','+','Z','T','R','A','N','S','F','E','R','=','1',',','2',',',
				         '1','0','0','0',',','1','0','0','0',0x0D,'\0'};
const int8_t ATEND[]={0x0D,'\0'};

const int8_t IPR[]={'A','T','+','I','P','R','=','9','6','0','0',0x0D,'\0'};//set baudrate to 9600
const int8_t ATsave[] = {'A','T','&','W',0x0D,'\0'};//保存配置，DT save the config
const int8_t ATres[] = {'A','T','&','F',0x0D,'\0'};//回复出厂设置，to defaule config

char ATPwrOff[] = {'A','T','+','Z','P','W','R','O','F','F',0x0D,'\0'};//关机，power off
char ATcsq[] = {'A','T','+','C','S','Q',0x0D,'\0'};//查信号强度，ask the intensity of signal
const int8_t ATifc[] = {'A','T','+','I','F','C','=','0',',','0',0x0D,'\0'};//禁用流控，TE-TA flow control,disable flow control 
const int8_t ATdtr[] = {'A','T','&','D','0',0x0D,'\0'};//忽略DTR,DTR signal,ignore DTR
const int8_t ATdcd[] = {'A','T','&','C','0',0x0D,'\0'};//CDC总是有效，DCD signal, DCD always effective
//const uint8_t ATbaud1[] = {'A','T','+','I','P','R','=','1','1','5','2','0','0',0x0D};

const int8_t ATbaud2[] = {'A','T','+','I','P','R','?',0x0D,'\0'};//ask baudrate at present



//const uint8_t ATpdp1[] = {'A','T','+','C','G','D','C','O','N','T','=','1',',',
//					'"','I','P','"',',','"','C','M','N','E','T','"'};//设置 PDP 格式
const int8_t ATpdp2[] = {'A','T','+','C','G','A','C','T','=','1',',','1',0x0D,'\0'};//激活 PDP 设置，1：激活，0：未激活
const int8_t ATcgatt[] = {'A','T','+','C','G','A','T','T','=','1',0x0D,'\0'};//设置 GPRS 业务，1:附着，：未附着
const int8_t ATclass[] = {'A','T','+','C','G','C','L','A','S','S','?',0x0D,'\0'};//查询GPRS等级

//GPRS参数
char ATzpnum[] = {'A','T','+','Z','P','N','U','M','=','"','C','M','N','E',
					'T','"',',','"','U','S','E','R','"',',','"','P','W','D','"',0x0D,'\0'};//接入点
char ATopen[] = {'A','T','+','Z','P','P','P','O','P','E','N',0x0D,'\0'};//打开 GPRS 连接
char ATclose[] = {'A','T','+','Z','P','P','P','C','L','O','S','E',0x0D,'\0'};//关闭 GPRS 连接
char ATgetip[] = {'A','T','+','Z','I','P','G','E','T','I','P',0X0D,'\0'};//获得模块的 IP 地址
//const uint8_t ATdnsip[] = {'A','T','+','Z','D','N','S','S','E','R','V','=','"',
//					'2','1','1',',','1','3','6',',','2','0',',','2',
 //                   '0','3','"',',','"','"'};//设置 DNS 服务器 IP 
//TCP连接
char ATlink[] = {'A','T','+','Z','I','P','S','E','T','U','P','=','1',',',0X0D,'\0'};//与目标服务器建立 TCP 连接
char ATsend[] = {'A','T','+','Z','I','P','S','E','N','D','=','1',',',0X0D,'\0'};//通过通道1，发送字节数据
//const uint8_t ATipcls[] = {'A','T','+','Z','I','P','C','L','O','S','E','=','1',};//关闭1通道TCP连接
char ATstu1[] = {'A','T','+','Z','P','P','P','S','T','A','T','U','S',0x0D,'\0'};//查询GPRS连接状态
char ATstu2[] = {'A','T','+','Z','I','P','S','T','A','T','U','S','=','1',0x0D,'\0'};//查询 TCP 连接状态
const int8_t ATIPClose[] = {'A','T','+','Z','I','P','C','L','O','S','E','=','1',0x0D,'\0'};//关闭当前 TCP 连接，1 通道
//const uint8_t ATrecv[] = {'+','Z','I','P','R','E','C','V',':','1',',',
 //                    '4','a','b','c','d'};//指定通道(1)接收制定数目(4)的字符

const int8_t IPBCDgs[12]={'1','2','4','1','2','8','0','0','0','0','9','8'};//公司IP地址的BCD码
const int8_t LPortIDgs[4]={'5','0','0','8'};//公司服务器端口号
//短信
const int8_t ATcnmi[] = {'A','T','+','C','N','M','I','=','3',',','1',',','0',',','0',',','0',0x0D,'\0'};//设置短信指令格式
const int8_t SetTEGSM[] = {'A','T','+','C','S','C','S','=','"','G','S','M','"',0x0D,'\0'};//TE字符集为GSM，14
const int8_t SetTEUCS2[] = {'A','T','+','C','S','C','S','=','"','U','C','S','2','"',0x0D,'\0'};//TE字符集为GSM，15
const int8_t AskTE[] = {'A','T','+','C','S','C','S','?',0x0D,'\0'};//查询TE字符集
const int8_t SetCNMsg[] = {'A','T','+','C','S','M','P','=','1','7',',','1','6','7',',','0',',','2','4','0',0x0D,'\0'};//中文短信,21
const int8_t ATcpms[] = {'A','T','+','C','P','M','S','=','"','S','M','"',0x0D,'\0'};//首选存储在SIM卡
const int8_t ATcmgf[] = {'A','T','+','C','M','G','F','=','1',0x0D,'\0'};//短信模式:0->PDU;1->文本
const int8_t ATcmgs[] = {'A','T','+','C','M','G','S','=','"','1','8','6','5','3','1','7','9',
					'2','1','6','"',0x0D,'\0'};//设置短信接收人
const int8_t ATcmgsCn[] = {'A','T','+','C','M','G','S','=','"','0','0','3','1','0','0','3','8','0','0','3','6',
					'0','0','3','5','0','0','3','3','0','0','3','1','0','0','3','7','0','0','3','9','0','0','3','2','0','0','3','1',
'0','0','3','6','"',0x0D,'\0'};//设置短信接收人,55
const int8_t message1[] = {'R','O','B','O','T','O','K',0x1A,0x0D,'\0'};//短信内容实例，ROBOTOK
const int8_t message2[] = {'T','h','a','n','k','Y','o','u',0x1A,0x0D,'\0'};//ThankYou

const int8_t tbl_ascii[]={48,49,50,51,52,53,54,55,56,57,65,66,67,68,69,70,};
							          // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, A, B, C, D, E, F, /
 
//定义私有变量
unsigned char DoNothing; //一个什么也不做的变量，实现NOP占位功能
unsigned char Clock1s; //1s定时器
unsigned char Clock100ms; //10ms定时器

#define Tx2BufMax 90
volatile  char  Tx2Buf[Tx2BufMax];//USART1发送数组 
#define Rx2BufMax 81
volatile  char  Rx2Buf[Rx2BufMax];//USART1接收数组
volatile char GPRSBuf[Tx2BufMax];//GSM模块临时数组

char GPRSCrc16Ascii[4];

volatile unsigned char  Tx2Read;
volatile unsigned char  CntTx2;//USART2发送计数器
volatile unsigned char  CntRx2;//USART2接收到的字节数 
volatile unsigned char  NbrOfDataToTrans2;//将要发送的字节数
volatile unsigned char  NbrOfDataToRead2;
volatile unsigned char  Rx2Start; //接收状态标志，0x55开始接收，0xAA接收完成
volatile unsigned char CntRec2Nms;//3.5字节计数器，超过3.5字节表示一帧数据接收完成
volatile unsigned char Cnt2DlyNms;//1帧接收完成后再延时3.5字节，防止影响其他MODBUS节点
volatile unsigned char  Rx2Read; //解析协议时读取数据的索引
volatile unsigned char  U1DrCopy2; //USART1接收器DR的备份
volatile unsigned char  ProUSART2; //协议解析允许标志，0x55允许解析，0xAA不允许解析
volatile unsigned char  NumPro2;//USART2等待解析的字节数
volatile unsigned char USART2ErrFlag; //串口1通讯故障标志：0x55发生通讯故障,0xAA通讯正常
volatile unsigned char USART2OrderMode; //命令类型：
volatile uint16_t USART2OrderWait; //超时计数器，向模块发送一次命令置成3，如果无回复，每个SysTick减1
#define  USART2OrderWaitNum 20000  //超时计数器的默认值
volatile unsigned char USART2OrderTimeOut; //Usart1OrderWait减到0时，Usart1OrderTimeOut+1
volatile unsigned char USART2OrderErr; //Usart1OrderTimeOut加到3时,置成0x55表示一次通讯故障,正常为0xAA 

extern volatile unsigned char  USARTErrData; //USART1或USART2收到的错误数据

//定义FlashEE数据
extern char DeviceID[12];//设备识别号
extern char APN[20];//EE中400~419，ASCII码型式
extern char USER[12];//EE中420~431，ASCII码型式
extern char PWD[8];//EE中432~439，ASCII码型式
extern char IPAddr[16];//EE中440~455，ASCII码型式
extern char PortID[4];//EE中456~459，ASCII码型式
extern char TimeAscii[];
extern char TimeAscii[11];//时间的ASCII码，WYYMMDDHHMM
extern char TimeAsciiNew[11];//RS485/GPRS接收到的时间的ASCII码，WYYMMDDHHMM
extern uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//rtc.c定义：接收到的设置，BCD码格式的日期时间
extern char VBtyAscii[];
extern char FTbkljAscii[];
extern char FTgkljAscii[];
extern volatile unsigned char FlagFTgklj;//1:精确到0.1m3，2:精确到1m3
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c中定义,工况累计流量,存储在EE484-487;工况瞬时流量,在EE中不存储
extern char FTbkssAscii[];
extern char FTgkssAscii[];
extern char PT1Ascii[];
extern char TTAscii[];//温度ASCII码
extern uint16_t NdValue;//计算之后的气体浓度
extern uint8_t BtyValue;//计算之后的电池电压值
extern uint8_t SelfStat;//设备自身状态：bit0-报警,bit1-传故,bit2-欠压,bit3-充电,bit4-冲完
extern uint8_t EEArray[];//对应488个模拟EE地址
extern char TranArray[];//发送前装填的数组,TW2.5
const uint8_t auchCRCLo[]=  
{  
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
	0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0xE, 0x0A, 0xCA, 0xCB, 0xB, 0xC9, 0x9, 0x8, 0xC8, 0xD8,
	0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 
	0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 
	0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 
	0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 
	0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 
	0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 
	0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 
	0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 
	0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 
	0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 
	0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 
	0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 
	0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40  
};  
  
const uint8_t auchCRCHi[] =  
{  
0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1,
	0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0,
	0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81,
	0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41,
	0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 
	0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0,
	0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81,
	0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40,
	0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1,
	0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0,
	0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81,
	0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41,
	0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1,
	0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0,
	0x80, 0x41, 0x0, 0xC1, 0x81, 0x40, 0x1, 0xC0, 0x80, 0x41, 0x1, 0xC0, 0x80, 0x41, 0x0, 0xC1, 0x81, 0x40   
};  

uint8_t crcstr[80];

void N_CRC16ASCII(char *updata,char len)  
{  
	uint8_t Crc16H = 0xFF,Crc16L,i;
	uint16_t index = 0;
	for (i = 0; i < len; i++)
	{
		index = (uint8_t)(Crc16L ^ updata[i]);
		Crc16L = (uint8_t)(Crc16H ^ auchCRCHi[index]);
		Crc16H = auchCRCLo[index];
	}
	GPRSCrc16Ascii[0]= tbl_ascii[(Crc16H>>4) & 0x0F];
	GPRSCrc16Ascii[1]= tbl_ascii[Crc16H & 0x0F];
	GPRSCrc16Ascii[2]= tbl_ascii[(Crc16L>>4) & 0x0F];
	GPRSCrc16Ascii[3]= tbl_ascii[Crc16L & 0x0F];
}  
//*----------------------------------------------------------
//函数名: Crc16Ascii
//功  能: 计算给定字符串的16位CRC码，并转化成4字节ASCII码
//输  入: num->字符串长度；Str[]->字符串
//输  出: GPRSCrc16Ascii[4]
//返回值: None
//*----------------------------------------------------------
void Crc16Ascii(char num,volatile char Str[])
{
	char Crch,Crcl;
	int16_t crc=0xFFFF;
	uint8_t crclinshi;
	char j,i;
//	crc16 =  cal_crc(Str, num);
	for(i=0;i<num;i++){
		crcstr[i]=Str[i];
		crc ^= crcstr[i];
		for(j=0;j<8;j++){
			crclinshi=crc&0x01;
			crc=crc>>1;
			if(crclinshi==1)
				crc^=0xA001;                
		} 
	}
	Crcl = (unsigned char)(crc & 0xFF);
	Crch = (unsigned char)((crc >> 8) & 0xFF);
	GPRSCrc16Ascii[0]= tbl_ascii[(Crch>>4) & 0x0F];
	GPRSCrc16Ascii[1]= tbl_ascii[Crch & 0x0F];
	GPRSCrc16Ascii[2]= tbl_ascii[(Crcl>>4) & 0x0F];
	GPRSCrc16Ascii[3]= tbl_ascii[Crcl & 0x0F];
}
uint8_t Ascii2Hex(uint8_t AsciiValue)//将ASCII码转换成数字
{
	uint8_t i;
	for(i=0;i<16;i++){//ASCII->BCD
		if(tbl_ascii[i]==AsciiValue)
		return i;
	}
	return 0;
}
void turnOnGSM(void)
{
	//uint8_t i,CntDly;
	EN4VOn();//delay(3000);//Open 4V
	GSMDown();
	DelayNms(220);//每个单位10ms,共2.2S,powerkey_n 3s 0v 
	GSMUp();

	DelayNms(1500);//每个单位10ms,共15S
}
void turOffGSM(void)
{
	GSMDown();
	DelayNms(220);//powerkey_n 4s 0v 
	GSMUp();
//	DelayNms(300);//powerkey_n 4s 0v
//	EN4VOff();//close 4v
}

void Uart2SendData(volatile uint8_t *Data,uint8_t DataLength)//向模块发送数据
{
	uint8_t i=0;

	for(i=0;i<DataLength;i++)
	{
		Tx2Buf[i] = Data[i];
	}
	NbrOfDataToTrans2 = DataLength;//将要发送的字节数
	Tx2Read=0;
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);//允许发送中断
}

void USART2SendStr(volatile char *str)//向模块发送字符串
{
	uint8_t IndexStr=0;
	while(str[IndexStr]!='\0'){
		Tx2Buf[IndexStr] = str[IndexStr];
		IndexStr++;
	}
	NbrOfDataToTrans2=IndexStr;
	Tx2Read=0;
	USART2OrderMode=1;//命令类型为1
	USART2OrderWait=USART2OrderWaitNum;//超时周期计数器100个SysTick收不到回复为一次超时
	//USART2OrderTimeOut=0;//当前
	USART2OrderErr=0xAA;//未发生通讯超时故障
	ProUSART2 = 0xAA;//不允许解析协议
	Rx2Start = 0xAA;//未接收到任何回复的数据
	for(IndexStr=0;IndexStr<Rx2BufMax;IndexStr++){//清空接收数组
		Rx2Buf[IndexStr]=0;
	}
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);//允许发送中断
}
void initGSM(void)
{
	uint8_t CntDly,DestChr;
	char *IndexRx2buf;
	DelayNms(1000);
	//1.循环发送"AT"，检查模块是否工作,正常应回复"OK"
	for(CntDly=0;CntDly<15;CntDly++){
		USART2SendStr(AT);
		DelayNms(100);//每个单位10ms,共1S//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(ProUSART2 == 0x55)  break ;//接到回复
	}
	if(CntDly>=14){//15次超时，说明GSM模块未工作
		__nop();
		turOffGSM();//bing gai
		DelayNms(1000);
		EN4VOff();
		DelayNms(500);
		turnOnGSM();
		DelayNms(500);//bing gai over
	}
	if(ProUSART2 == 0x55){//收到回复数据
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){
			__nop();//调试观察是否收到OK
		}
	}
	//2.发送"ATE0"关闭回显,应回复"ATE0","OK"
	USART2SendStr(ATE0);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到OK
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){
			__nop();
		}
	}
	//3.发送"AT+CPIN?"查询SIM卡状态,正常应回复"+CPIN: READY","OK"
	USART2SendStr(ATCPIN);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//收到OK,卡正常
			__nop();
		}else{//无卡
			__nop();//LED指示无卡
		}
	}
	//4.发送"AT+CREG?"查询基站网络,正常应回复"+CREG: X,Y","OK"  X->MODE  Y->STAT
	USART2SendStr(ATCREG);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		__nop();
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//收到OK
			IndexRx2buf=strchr((const char*)Rx2Buf,' ');//char *strchr(const char *,int);
			if(IndexRx2buf!=NULL){
				DestChr=IndexRx2buf[1];//0:禁止提示;1:使能提示，格式:+CREG: X,Y;2:使能提示，格式:+CREG: X,Y[A,B,C]
			}
			IndexRx2buf=strchr((const char*)Rx2Buf,',');
			if(IndexRx2buf!=NULL){
				DestChr=IndexRx2buf[2];//0:未注册; 1:已注册; 2:未注册，正搜寻基站;4:未知代码;5:已注册,漫游
			}	
		}//收到OK
	}//收到回复
	//5.发送"AT+CSQ"查询信号强度,正常应回复"+CSQ: XX, XX","OK"
	USART2SendStr(ATCSQ);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//收到OK
			__nop();
		}//收到OK
	}//收到回复
	//6.发送"AT+CGATT?"查询GPRS附着状态,正常应回复"+CGATT:X","OK"
	USART2SendStr(ATCGATT);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		//if(strstr((const char*)Rx2Buf,"OK")!='\0'){} //收到OK
		IndexRx2buf=strstr((const char*)Rx2Buf,"ATT:");
		if(IndexRx2buf!=NULL){//收到+CGATT:X
			DestChr=IndexRx2buf[5];
			if(DestChr=='0'){//未附着
				USART2SendStr(ATCGATT1);//发送AT+CGATT=1,强制附着GPRS
				while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
					__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				}
				if(USART2OrderErr == 0x55){ //超时 
					__nop();
				}
				if(strstr((const char*)Rx2Buf,"OK")!='\0'){//收到OK
					__nop();
				} 
			}//未附着
		}//收到+CGATT:X
	}
	
}
uint8_t  UDPLink(void)
{
	uint8_t i,NumGPRS;
//1.发送"AT+ZPNUM="CMNET","USER","PWD"",设置APN名称\用户名\密码,正常应回复"OK"
//	char ZPNum[]={'A','T','+','Z','P','N','U','M','=','"','C','M','N','E','T','"',
//				      ',','"','U','S','E','R','"',',','"','P','W','D','"',0x0D,'\0'};
	//添加载入APN的操作
	for(i=0;i<10;i++) GPRSBuf[i]=ZPNum[i];//1>AT+ZPNUM="
	i=0;NumGPRS=10;
	while(APN[i]!='\0'){//2>装入APN
		GPRSBuf[NumGPRS++]=APN[i++];//char APN[20];//EE中400~419，ASCII码型式
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=','; GPRSBuf[NumGPRS++]='"';
	i=0;
	while(USER[i]!='\0'){//3>装入USER
		GPRSBuf[NumGPRS++]=USER[i++];//char USER[12];//EE中420~431，ASCII码型式
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=','; GPRSBuf[NumGPRS++]='"';
	i=0;
	while(PWD[i]!='\0'){//4>装入PWD
		GPRSBuf[NumGPRS++]=PWD[i++];//char PWD[8];//EE中432~439，ASCII码型式
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=0x0D; GPRSBuf[NumGPRS++]='\0';
	//USART2SendStr(ZPNum);
	USART2SendStr(GPRSBuf);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"OK")){//接收到OK
			__nop();
		}
	}
	//2.发送"AT+ZPPPOPEN,打开GPRS连接,正常应回复"+ZPPPOPEN:CONNECTED""OK"
	USART2SendStr(ZPPPOpen);
	DelayNms(120);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"CONNECTED")){//接收到+ZPPPPOPEN:CONNECTED
			__nop();
		}
	}//收到回复
	//3.发送"AT+ZIPSETUPU=1,***.***.***.***,5008,打开UDP连接,正常应回复"+ZIPSETUPU:CONNECTED" "OK"
	//添加载入IP地址和端口号的操作
	for(i=0;i<14;i++) GPRSBuf[i]=SETUP[i];//1>AT+ZIPSETUPU=1,
	i=0;NumGPRS=14;
	while(IPAddr[i]!='\0'){//2>装入IPAddr[16]
		GPRSBuf[NumGPRS++]=IPAddr[i++];//char IPAddr[16];//EE中440~455，ASCII码型式
	}
	GPRSBuf[NumGPRS++]=',';
	i=0;
	for(i=0;i<4;i++){//3>装入PortID[4],共4个字节
		GPRSBuf[NumGPRS++]=PortID[i];//char PortID[4];//EE中456~459，ASCII码型式
	}
	GPRSBuf[NumGPRS++]=0x0D; GPRSBuf[NumGPRS++]='\0';
	USART2SendStr(GPRSBuf);
	DelayNms(100);//bing gai
	//USART2SendStr(SETUPU);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"CONNECTED")){//接收到+ZIPSETUPU:CONNECTED
			__nop();
		}
	}//收到回复
	//4.发送AT+ZTRANSFER=1,1,1000,1000,正常应回复"+ZTRANSFER:1""OK"
	//1:SOCKET1  2:TCP(1:UDP)  1000:1000ms超时,   1000:每帧最大1000BYTE
	USART2SendStr(TRANSFER_TCP);
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"TRANSFER:1")){//接收到+ZTRANSFER:1
			__nop();
		}
	}//收到回复
	//5.发送ATO,正常应回复"Enter into data mode, please input data:""OK"
	USART2SendStr(AT2DATA);
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"OK")){//接收到+ZTRANSFER:1
			__nop();
			return 1;//成功连接
		}
	}//收到回复
	return 0;//连接失败
}
uint8_t GPRSHisTrans(uint8_t num)//GPRS发送历史数据
{
	uint32_t FlagZhen,DataEE;
	uint16_t IndexEE;
	uint8_t Num;
	uint8_t FlagHis=0;//发送成功标志，成功时置成1
	
	Num=num;
	FlagZhen=FlashEERead(Num*100+88);//检查帧标志位
	if((FlagZhen & 0x000000FF)==0x01){//帧满
		for(IndexEE=Num*100;IndexEE<Num*100+88;IndexEE+=4){//将本帧数据装入GPRSBuf[],IndexEE需加上帧偏移量
			DataEE=FlashEERead(IndexEE);
			GPRSBuf[IndexEE-Num*100]=(char)(DataEE & 0x000000FF);//无论第几帧，GPRSBuf[]都从0开始
			GPRSBuf[IndexEE-Num*100+1]=(char)((DataEE>>8) & 0x000000FF);
			GPRSBuf[IndexEE-Num*100+2]=(char)((DataEE>>16) & 0x000000FF);
			GPRSBuf[IndexEE-Num*100+3]=(char)((DataEE>>24) & 0x000000FF);
		}
		FlagHis=0;
		USART2SendStr(GPRSBuf);//第一次发送,历史数据仅发送一次
		DelayNms(150);//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(USART2OrderErr == 0x55){ //超时 
			__nop();
		}
		if(ProUSART2 == 0x55){ //收到回复
			ProUSART2 = 0xAA;__nop();
			if(strstr((const char *)Rx2Buf,"SUCCESS")){//接收到服务器回复:##SUCCESSCRC@@,CRC是4字节ASCII码
				FlagHis=1;//服务器已收到数据
				FlashEEWrite(num*100+88,0);//清零帧标志位
				__nop();
			}
		}
	}else{//帧空
		FlagHis=2;//帧空时返回2
	}
	return FlagHis;
}
uint8_t GPRSProTrans(uint8_t Flagtrans)
{
	uint8_t IndexTrans,i;
	uint8_t FlagTrans=0;//发送成功标志，成功时置成1
	//uint8_t NumZhen;	
	//uint32_t FlagZhen,DataEE;
	char DeviceIP[18]={'I','P',':','1','2','3','.','2','3','.','4','3','.','3','4','#','#','\0'};
	char *IndexWL,*IndexWT,*IndexW1;
	//1.上传采集的传感器数据
	//@@05310000001IP:123.23.43.34##A0000Z
	GPRSBuf[0]='@';GPRSBuf[1]='@';//1>封装包头
	for(IndexTrans=0;IndexTrans<11;IndexTrans++){//2>搬运ID,exp:05310000001
		GPRSBuf[IndexTrans+2]=DeviceID[IndexTrans];//第2-12字节
	}
	for(IndexTrans=0;IndexTrans<17;IndexTrans++){//3>搬运IP,exp:IP:123.23.43.34##
		GPRSBuf[IndexTrans+13]=DeviceIP[IndexTrans];//第13-29字节
	}
	for(IndexTrans=0;IndexTrans<6;IndexTrans++){//3>搬运数据 TranArray[]
		GPRSBuf[IndexTrans+30]=TranArray[IndexTrans];//第30-35字节
	}
	
//	GPRSBuf[36]=0x1A;//4>封装包尾
	GPRSBuf[36]='\0';
	
	FlagTrans=0;
	//USART2SendStr("ABCD");//测试用
	USART2SendStr(GPRSBuf);//第一次发送
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //超时 
		__nop();
	}
	if(ProUSART2 == 0x55){ //收到回复
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"A") && strchr((const char *)Rx2Buf,'Z')){//接收到服务器回复:##SUCCESSCRC@@,CRC是4字节ASCII码
			FlagTrans=1;//服务器已收到数据
			IndexWL=strstr((const char *)Rx2Buf,"WL");
			if(IndexWL!=NULL){//配置累积流量
				FTgkljAscii[1] = IndexWL[2];
				FTgkljAscii[2] = IndexWL[3];
				FTgkljAscii[3] = IndexWL[4];
				FTgkljAscii[4] = IndexWL[5];
				FTgkljAscii[5] = IndexWL[6];
				FTgkljAscii[6] = IndexWL[7];
				FTgkljAscii[7] = IndexWL[8];
				FTgkljAscii[8] = IndexWL[9];
				FTgklj = 0xF0000000 & (Ascii2Hex(IndexWL[2])<<28);
				FTgklj |=  0x0F000000 & (Ascii2Hex(IndexWL[3])<<24);
				FTgklj |=  0x00F00000 & (Ascii2Hex(IndexWL[4])<<20);
				FTgklj |=  0x000F0000 & (Ascii2Hex(IndexWL[5])<<16);
				FTgklj |=  0x0000F000 & (Ascii2Hex(IndexWL[6])<<12);
				FTgklj |=  0x00000F00 & (Ascii2Hex(IndexWL[7])<<8);
				FTgklj |=  0x000000F0 & (Ascii2Hex(IndexWL[8])<<4);
				FTgklj |=  0x0000000F & Ascii2Hex(IndexWL[9]);
				EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
				EEArray[486]=(FTgklj>>16) & 0xFF;
				RefreshEE();
					
				USART2SendStr("%WLOK$");////回复配置信息
			}//if(IndexWL!=NULL){//配置累积流量
			IndexWT=strstr((const char *)Rx2Buf,"WT");
			if(IndexWT!=NULL){//配置时间
				for(i=0;i<11;i++){
					TimeAsciiNew[i]=IndexWT[i+2];
				}
				//添加设置RTC程序段
				NewWeek = Ascii2Hex(TimeAsciiNew[0]);
				NewYear = (Ascii2Hex(TimeAsciiNew[1])<<4) & 0xF0;
				NewYear|= Ascii2Hex(TimeAsciiNew[2]) & 0x0F;
				NewMonth = (Ascii2Hex(TimeAsciiNew[3])<<4) & 0xF0;
				NewMonth|= Ascii2Hex(TimeAsciiNew[4]) & 0x0F;
				NewDate = (Ascii2Hex(TimeAsciiNew[5])<<4) & 0xF0;
				NewDate|= Ascii2Hex(TimeAsciiNew[6]) & 0x0F;
				NewHours = (Ascii2Hex(TimeAsciiNew[7])<<4) & 0xF0;
				NewHours|= Ascii2Hex(TimeAsciiNew[8]) & 0x0F;
				NewMinutes = (Ascii2Hex(TimeAsciiNew[9])<<4) & 0xF0;
				NewMinutes|= Ascii2Hex(TimeAsciiNew[10]) & 0x0F;
				NewSeconds = 0x30;//设置在30S，时钟误差最低
				ConfigTime(NewHours,NewMinutes,NewSeconds);
				ConfigDate(NewWeek,NewYear,NewMonth,NewDate);
				
				USART2SendStr("%WTOK$");////回复配置信息
			}//if(IndexWT!=NULL){//配置时间
			IndexW1=strstr((const char *)Rx2Buf,"W1");
			if(IndexW1!=NULL){//配置流量单位标志
				if(IndexW1[2]=='1')
					FlagFTgklj=1;
				else if(IndexW1[2]=='2')
					FlagFTgklj=2;
				else 
					FlagFTgklj=1;
				EEArray[481]=FlagFTgklj;
				RefreshEE();				
				USART2SendStr("%W1OK$");////回复配置信息
			}//if(IndexW1!=NULL){//配置流量单位标志
			__nop();
		}
	}
	/*
	if(FlagTrans==0){//服务器未成功收到数据
		USART2SendStr(GPRSBuf);//重发
		DelayNms(50);//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //等待接收完数据，并且进行超时判断
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(USART2OrderErr == 0x55){ //超时 
			__nop();
		}
		if(ProUSART2 == 0x55){ //收到回复
			ProUSART2 = 0xAA;__nop();
			if(strstr((const char *)Rx2Buf,"A") && strchr((const char *)Rx2Buf,'Z')){//接收到服务器回复:##SUCCESSCRC@@,CRC是4字节ASCII码
				FlagTrans=1;//服务器已收到数据
				IndexWL=strstr((const char *)Rx2Buf,"WL");
				if(IndexWL!=NULL){//配置累积流量
					FTgkljAscii[1] = IndexWL[2];
					FTgkljAscii[2] = IndexWL[3];
					FTgkljAscii[3] = IndexWL[4];
					FTgkljAscii[4] = IndexWL[5];
					FTgkljAscii[5] = IndexWL[6];
					FTgkljAscii[6] = IndexWL[7];
					FTgkljAscii[7] = IndexWL[8];
					FTgkljAscii[8] = IndexWL[9];
					FTgklj = 0xF0000000 & (Ascii2Hex(IndexWL[2])<<28);
					FTgklj |=  0x0F000000 & (Ascii2Hex(IndexWL[3])<<24);
					FTgklj |=  0x00F00000 & (Ascii2Hex(IndexWL[4])<<20);
					FTgklj |=  0x000F0000 & (Ascii2Hex(IndexWL[5])<<16);
					FTgklj |=  0x0000F000 & (Ascii2Hex(IndexWL[6])<<12);
					FTgklj |=  0x00000F00 & (Ascii2Hex(IndexWL[7])<<8);
					FTgklj |=  0x000000F0 & (Ascii2Hex(IndexWL[8])<<4);
					FTgklj |=  0x0000000F & Ascii2Hex(IndexWL[9]);
					EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
					EEArray[486]=(FTgklj>>16) & 0xFF;
					RefreshEE();					
					USART2SendStr("%WLOK$");////回复配置信息
				}//if(IndexWL!=NULL){//配置累积流量
				IndexWT=strstr((const char *)Rx2Buf,"WT");
				if(IndexWT!=NULL){//配置时间
					for(i=0;i<11;i++){
						TimeAsciiNew[i]=IndexWT[i+2];
					}
					//添加设置RTC程序段
					NewWeek = Ascii2Hex(TimeAsciiNew[0]);
					NewYear = (Ascii2Hex(TimeAsciiNew[1])<<4) & 0xF0;
					NewYear|= Ascii2Hex(TimeAsciiNew[2]) & 0x0F;
					NewMonth = (Ascii2Hex(TimeAsciiNew[3])<<4) & 0xF0;
					NewMonth|= Ascii2Hex(TimeAsciiNew[4]) & 0x0F;
					NewDate = (Ascii2Hex(TimeAsciiNew[5])<<4) & 0xF0;
					NewDate|= Ascii2Hex(TimeAsciiNew[6]) & 0x0F;
					NewHours = (Ascii2Hex(TimeAsciiNew[7])<<4) & 0xF0;
					NewHours|= Ascii2Hex(TimeAsciiNew[8]) & 0x0F;
					NewMinutes = (Ascii2Hex(TimeAsciiNew[9])<<4) & 0xF0;
					NewMinutes|= Ascii2Hex(TimeAsciiNew[10]) & 0x0F;
					NewSeconds = 0x30;//设置在30S，时钟误差最低
					ConfigTime(NewHours,NewMinutes,NewSeconds);
					ConfigDate(NewWeek,NewYear,NewMonth,NewDate);			
					USART2SendStr("%WTOK$");////回复配置信息
					__nop();	
				}//if(IndexWT!=NULL){//配置时间
				IndexW1=strstr((const char *)Rx2Buf,"W1");
				if(IndexW1!=NULL){//配置流量单位标志
					if(IndexW1[2]=='1')
						FlagFTgklj=1;
					else if(IndexW1[2]=='2')
						FlagFTgklj=2;
					else 
						FlagFTgklj=1;
					EEArray[481]=FlagFTgklj;
					RefreshEE();			
					USART2SendStr("%W1OK$");////回复配置信息
				}//if(IndexW1!=NULL){//配置流量单位标志
			}//if(strstr((const char *)Rx2Buf,"SUCCESS")){
		}//if(ProUSART2 == 0x55){ //收到回复
	}//if(FlagTrans==0){//服务器未成功收到数据
	//存储发送不成功的帧
	*/
	if(FlagTrans==0){//服务器第2次仍未成功收到数据,存储本次未发送成功的数据
		/*
		for(NumZhen=0;NumZhen<4;NumZhen++){
			FlagZhen=FlashEERead(NumZhen*100+88);
			if(FlagZhen){//当前帧满
				if(NumZhen==3){//0-3帧都满
					__nop();//放弃存储未发送数据
				}
			}else{//当前帧空
				for(IndexTrans=0;IndexTrans<88;IndexTrans+=4){//将数据装入当前帧
					DataEE = (((uint32_t)GPRSBuf[IndexTrans+3]) << 24) & 0xFF000000;
					DataEE |= (((uint32_t)GPRSBuf[IndexTrans+2]) << 16) & 0x00FF0000;
					DataEE |= (((uint32_t)GPRSBuf[IndexTrans+1]) << 8) & 0x0000FF00;
					DataEE |= ((uint32_t)GPRSBuf[IndexTrans]) & 0x000000FF;
					FlashEEWrite(NumZhen*100+IndexTrans,DataEE);
				}
				FlashEEWrite(NumZhen*100+88,1);//当前帧标志改成1：满
				while(NumZhen<4){
					NumZhen++;
					FlashEEWrite(NumZhen*100+88,0);//将当前帧之后的帧标志全部改成空
				}
			}//当前帧空
		}
		*/
		return 0;
	}
	return 1;
}
void ComUDP(int8_t FlagUDP)//与服务器UDP通讯
{
	//int8_t FlagUdp;
	uint8_t FlagPro;
	
	Reinit_RCC_HSE();
	Systick_Init();
	Init_USART2();
	USART2VarInit();
	turnOnGSM();
	initGSM();
	FlagPro=UDPLink();
	if(FlagPro==0) 
		FlagPro=UDPLink();//第二次连接
	
	FlagPro=GPRSProTrans(FlagUDP);
	if(FlagPro==0){
		turOffGSM();
		DelayNms(1500);//powerkey_n 15s 0v
		turnOnGSM();
		initGSM();
		FlagPro=UDPLink();
		FlagPro=GPRSProTrans(FlagUDP);
	}
	//DelayNms(200);//每个单位10ms
	//USART2SendStr(AT2ORDER);
	//DelayNms(200);//每个单位10ms
	//USART2SendStr(ATPwrOff);
	//DelayNms(100);//每个单位10ms
	turOffGSM();
	DelayNms(1000);//powerkey_n 10s 0v
	CloseUSART2();
	Reinit_RCC_HSI();
	Systick_Init();
	Init_GPIO();
}






//********************************USART2程序*****************************
void USART2VarInit(void)
{
	Tx2Read = 0;
	CntRx2 = 0; 
	NbrOfDataToTrans2 = 0;
	NbrOfDataToRead2 = 0;
	Rx2Start = 0xAA;//接收状态标志，0x55开始接收，0xAA接收完成
	Rx2Read = 0;//解析协议时读取数据的索引
	U1DrCopy2 = 0;//USART1接收器DR的备份

	ProUSART2 = 0xAA;//0x55允许解析数据,0xAA不允许解析数据
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//用库函数禁止发送中断
	USART2ErrFlag = 0xAA; //无故障
	USART2OrderWait = 0; //没有正在等待的命令
	USART2OrderMode = 1;//命令类型为1
	
	USARTErrData = 0;//接收错误时的数据字节
}
void Init_USART2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//TX2:GPIOA_2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//RX2:GPIOA_3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE ); 
	USART_InitStructure.USART_BaudRate = 115200;
	//USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE,ENABLE);//允许接收中断
	USART_ITConfig(USART2, USART_IT_TXE,DISABLE);//禁止发送中断
	USART_Cmd(USART2, ENABLE);
}
 
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		if(Rx2Start == 0x55){	//接收过程已经开始，本字节不是第一个字节
			DoNothing = 1;//调试占位，设置断点用
			CntRec2Nms = 0;	//清3.5字节超时计数器
			//Rx2Buf[CntRx2] = USART_ReceiveData(USART2);
			Rx2Buf[CntRx2] = (uint8_t)(USART2->RDR & (uint8_t)0xFF);
			DoNothing = 1;
			CntRx2++;
			if(CntRx2 > 50){
				CntRx2 = 0;	//接收到超过50个字节的帧，协议中无超过50字节的帧
				Rx2Start = 0xAA;
			}
		}else{	//接收第一个字节
			CntRec2Nms = 0;
			Rx2Start = 0x55;//接收正式开始
			CntRx2 = 0;
			Rx2Buf[CntRx2++] = (uint8_t)(USART2->RDR & (uint8_t)0xFF);
		}
		//接收错误处理
		if(USART_GetFlagStatus(USART2, USART_FLAG_ORE)==SET)//溢出错误
		{USART_ClearFlag(USART2, USART_FLAG_ORE);}
		if(USART_GetFlagStatus(USART2, USART_FLAG_FE)==SET)//帧错误
		{USART_ClearFlag(USART2, USART_FLAG_FE);}
   	if(USART_GetFlagStatus(USART2, USART_FLAG_PE)==SET)//校验错误
		{USART_ClearFlag(USART2, USART_FLAG_PE);}		
	}//接收中断程序结束
	
	if(USART_GetITStatus(USART2,USART_IT_TXE) == ENABLE)
	{
		if(NbrOfDataToTrans2==0)//发送完成
		{
			//USART1->CR1 &= ~(1 << 7);//操作寄存器禁止发送中断
			//GPIOA->ODR &= ~(1<<8); //操作寄存器将485总线改成接收状态�
			USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//用库函数禁止发送中断
			//GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);//用库函数485总线改成接收状态
		}else{
			//USART1->TDR = (uint16_t)Tx1Buf[Tx1Read++] & (uint16_t)0x01FF;
			//USART2->TDR = Tx2Buf[Tx2Read++];
			USART_SendData(USART2, (uint16_t)(Tx2Buf[Tx2Read++])&0x01FF);
			NbrOfDataToTrans2--;
		}
	}
}
void CloseUSART2(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;

	USART_Cmd(USART2, DISABLE);//关闭USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,DISABLE );
	//配置PA2\PA3以节能
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//TX2\RX2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	
	//关闭MG2639电源
	EN4VOff();
	GSMUp();
}
//*------------------------------------------------
//* 函数名: USART2Trans
//* 作  用：USART2发送控制函数
//* 参  数: String，发送数据数组名
//* 返回值: None
//*------------------------------------------------
void USART2Trans(void)
{
	Tx2Buf[0]=0x55;Tx2Buf[1]=0xAA;Tx2Buf[2]=0x36;Tx2Buf[3]=0x39;
	NbrOfDataToTrans2=4;Tx2Read=0;
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
	//USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//发送完成后在中断中关发送中断
}
/************************ (C) 版权 邵开春 *************************/
