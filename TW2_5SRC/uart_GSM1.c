/*****************************************************************************
  *�ļ���: F051test/uart_GSM.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/06
  *��@@����ͨ��uart2��GPRSģ��ͨѶ
********************************************************************************/
#include "main.h"
#include "string.h"
#include "stm32f0xx_it.h" //�ж�ͷ�ļ���stm32f0xx_conf.h��δ����

//����˽��define

//GSM Const
char AT2ORDER[] = {'+','+','+',0x0D,'\0'};//����ģʽת��������ģʽ��change data mode to order mode
char AT2DATA[] = {'A','T','O',0x0D,'\0'};//����ģʽת��������ģʽ��change order mode to data mode
//char AT2DATA[] = {'A','T','O','\0'};//����ģʽת��������ģʽ��change order mode to data mode
const int8_t ATsimstu[] = {'A','T','*','T','S','I','M','I','N','S','?','\0'};//��SIM��״̬��ask the simcard status
const int8_t ATcops1[] = {'A','T','+','C','O','P','S','?','\0'};//������״̬��ask the net status
//const uint8_t ATcops2[] = {'A','T','+','C','O','P','S','=','4',',','0',',','"',
//					'C','h','i','n','a',' ','M','o','b','i','l','e','"'};//select net 

char AT[]={'A','T',0X0D,'\0'};              //�ַ����Ľ�βΪ '/0'
char ATE0[]={'A','T','E','0',0X0D,'\0'};    //�رջ���
char ATCPIN[]={'A','T','+','C','P','I','N','?',0X0D,'\0'};    //�鿴SIM��״̬
char ATCREG[]={'A','T','+','C','R','E','G','?',0X0D,'\0'};    //�鿴GSM����ע��״̬
char ATCSQ[]={'A','T','+','C','S','Q',0X0D,'\0'};    //�鿴�ź�ǿ��
char ATCGATT[]={'A','T','+','C','G','A','T','T','?',0X0D,'\0'};//��ѯGPRS����״̬
char ATCGATT1[]={'A','T','+','C','G','A','T','T','=','1',0X0D,'\0'};//ǿ��GPRS����
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
const int8_t ATsave[] = {'A','T','&','W',0x0D,'\0'};//�������ã�DT save the config
const int8_t ATres[] = {'A','T','&','F',0x0D,'\0'};//�ظ��������ã�to defaule config

char ATPwrOff[] = {'A','T','+','Z','P','W','R','O','F','F',0x0D,'\0'};//�ػ���power off
char ATcsq[] = {'A','T','+','C','S','Q',0x0D,'\0'};//���ź�ǿ�ȣ�ask the intensity of signal
const int8_t ATifc[] = {'A','T','+','I','F','C','=','0',',','0',0x0D,'\0'};//�������أ�TE-TA flow control,disable flow control 
const int8_t ATdtr[] = {'A','T','&','D','0',0x0D,'\0'};//����DTR,DTR signal,ignore DTR
const int8_t ATdcd[] = {'A','T','&','C','0',0x0D,'\0'};//CDC������Ч��DCD signal, DCD always effective
//const uint8_t ATbaud1[] = {'A','T','+','I','P','R','=','1','1','5','2','0','0',0x0D};

const int8_t ATbaud2[] = {'A','T','+','I','P','R','?',0x0D,'\0'};//ask baudrate at present



//const uint8_t ATpdp1[] = {'A','T','+','C','G','D','C','O','N','T','=','1',',',
//					'"','I','P','"',',','"','C','M','N','E','T','"'};//���� PDP ��ʽ
const int8_t ATpdp2[] = {'A','T','+','C','G','A','C','T','=','1',',','1',0x0D,'\0'};//���� PDP ���ã�1�����0��δ����
const int8_t ATcgatt[] = {'A','T','+','C','G','A','T','T','=','1',0x0D,'\0'};//���� GPRS ҵ��1:���ţ���δ����
const int8_t ATclass[] = {'A','T','+','C','G','C','L','A','S','S','?',0x0D,'\0'};//��ѯGPRS�ȼ�

//GPRS����
char ATzpnum[] = {'A','T','+','Z','P','N','U','M','=','"','C','M','N','E',
					'T','"',',','"','U','S','E','R','"',',','"','P','W','D','"',0x0D,'\0'};//�����
char ATopen[] = {'A','T','+','Z','P','P','P','O','P','E','N',0x0D,'\0'};//�� GPRS ����
char ATclose[] = {'A','T','+','Z','P','P','P','C','L','O','S','E',0x0D,'\0'};//�ر� GPRS ����
char ATgetip[] = {'A','T','+','Z','I','P','G','E','T','I','P',0X0D,'\0'};//���ģ��� IP ��ַ
//const uint8_t ATdnsip[] = {'A','T','+','Z','D','N','S','S','E','R','V','=','"',
//					'2','1','1',',','1','3','6',',','2','0',',','2',
 //                   '0','3','"',',','"','"'};//���� DNS ������ IP 
//TCP����
char ATlink[] = {'A','T','+','Z','I','P','S','E','T','U','P','=','1',',',0X0D,'\0'};//��Ŀ����������� TCP ����
char ATsend[] = {'A','T','+','Z','I','P','S','E','N','D','=','1',',',0X0D,'\0'};//ͨ��ͨ��1�������ֽ�����
//const uint8_t ATipcls[] = {'A','T','+','Z','I','P','C','L','O','S','E','=','1',};//�ر�1ͨ��TCP����
char ATstu1[] = {'A','T','+','Z','P','P','P','S','T','A','T','U','S',0x0D,'\0'};//��ѯGPRS����״̬
char ATstu2[] = {'A','T','+','Z','I','P','S','T','A','T','U','S','=','1',0x0D,'\0'};//��ѯ TCP ����״̬
const int8_t ATIPClose[] = {'A','T','+','Z','I','P','C','L','O','S','E','=','1',0x0D,'\0'};//�رյ�ǰ TCP ���ӣ�1 ͨ��
//const uint8_t ATrecv[] = {'+','Z','I','P','R','E','C','V',':','1',',',
 //                    '4','a','b','c','d'};//ָ��ͨ��(1)�����ƶ���Ŀ(4)���ַ�

const int8_t IPBCDgs[12]={'1','2','4','1','2','8','0','0','0','0','9','8'};//��˾IP��ַ��BCD��
const int8_t LPortIDgs[4]={'5','0','0','8'};//��˾�������˿ں�
//����
const int8_t ATcnmi[] = {'A','T','+','C','N','M','I','=','3',',','1',',','0',',','0',',','0',0x0D,'\0'};//���ö���ָ���ʽ
const int8_t SetTEGSM[] = {'A','T','+','C','S','C','S','=','"','G','S','M','"',0x0D,'\0'};//TE�ַ���ΪGSM��14
const int8_t SetTEUCS2[] = {'A','T','+','C','S','C','S','=','"','U','C','S','2','"',0x0D,'\0'};//TE�ַ���ΪGSM��15
const int8_t AskTE[] = {'A','T','+','C','S','C','S','?',0x0D,'\0'};//��ѯTE�ַ���
const int8_t SetCNMsg[] = {'A','T','+','C','S','M','P','=','1','7',',','1','6','7',',','0',',','2','4','0',0x0D,'\0'};//���Ķ���,21
const int8_t ATcpms[] = {'A','T','+','C','P','M','S','=','"','S','M','"',0x0D,'\0'};//��ѡ�洢��SIM��
const int8_t ATcmgf[] = {'A','T','+','C','M','G','F','=','1',0x0D,'\0'};//����ģʽ:0->PDU;1->�ı�
const int8_t ATcmgs[] = {'A','T','+','C','M','G','S','=','"','1','8','6','5','3','1','7','9',
					'2','1','6','"',0x0D,'\0'};//���ö��Ž�����
const int8_t ATcmgsCn[] = {'A','T','+','C','M','G','S','=','"','0','0','3','1','0','0','3','8','0','0','3','6',
					'0','0','3','5','0','0','3','3','0','0','3','1','0','0','3','7','0','0','3','9','0','0','3','2','0','0','3','1',
'0','0','3','6','"',0x0D,'\0'};//���ö��Ž�����,55
const int8_t message1[] = {'R','O','B','O','T','O','K',0x1A,0x0D,'\0'};//��������ʵ����ROBOTOK
const int8_t message2[] = {'T','h','a','n','k','Y','o','u',0x1A,0x0D,'\0'};//ThankYou

const int8_t tbl_ascii[]={48,49,50,51,52,53,54,55,56,57,65,66,67,68,69,70,};
							          // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, A, B, C, D, E, F, /
 
//����˽�б���
unsigned char DoNothing; //һ��ʲôҲ�����ı�����ʵ��NOPռλ����
unsigned char Clock1s; //1s��ʱ��
unsigned char Clock100ms; //10ms��ʱ��

#define Tx2BufMax 90
volatile  char  Tx2Buf[Tx2BufMax];//USART1�������� 
#define Rx2BufMax 81
volatile  char  Rx2Buf[Rx2BufMax];//USART1��������
volatile char GPRSBuf[Tx2BufMax];//GSMģ����ʱ����

char GPRSCrc16Ascii[4];

volatile unsigned char  Tx2Read;
volatile unsigned char  CntTx2;//USART2���ͼ�����
volatile unsigned char  CntRx2;//USART2���յ����ֽ��� 
volatile unsigned char  NbrOfDataToTrans2;//��Ҫ���͵��ֽ���
volatile unsigned char  NbrOfDataToRead2;
volatile unsigned char  Rx2Start; //����״̬��־��0x55��ʼ���գ�0xAA�������
volatile unsigned char CntRec2Nms;//3.5�ֽڼ�����������3.5�ֽڱ�ʾһ֡���ݽ������
volatile unsigned char Cnt2DlyNms;//1֡������ɺ�����ʱ3.5�ֽڣ���ֹӰ������MODBUS�ڵ�
volatile unsigned char  Rx2Read; //����Э��ʱ��ȡ���ݵ�����
volatile unsigned char  U1DrCopy2; //USART1������DR�ı���
volatile unsigned char  ProUSART2; //Э����������־��0x55���������0xAA���������
volatile unsigned char  NumPro2;//USART2�ȴ��������ֽ���
volatile unsigned char USART2ErrFlag; //����1ͨѶ���ϱ�־��0x55����ͨѶ����,0xAAͨѶ����
volatile unsigned char USART2OrderMode; //�������ͣ�
volatile uint16_t USART2OrderWait; //��ʱ����������ģ�鷢��һ�������ó�3������޻ظ���ÿ��SysTick��1
#define  USART2OrderWaitNum 20000  //��ʱ��������Ĭ��ֵ
volatile unsigned char USART2OrderTimeOut; //Usart1OrderWait����0ʱ��Usart1OrderTimeOut+1
volatile unsigned char USART2OrderErr; //Usart1OrderTimeOut�ӵ�3ʱ,�ó�0x55��ʾһ��ͨѶ����,����Ϊ0xAA 

extern volatile unsigned char  USARTErrData; //USART1��USART2�յ��Ĵ�������

//����FlashEE����
extern char DeviceID[12];//�豸ʶ���
extern char APN[20];//EE��400~419��ASCII����ʽ
extern char USER[12];//EE��420~431��ASCII����ʽ
extern char PWD[8];//EE��432~439��ASCII����ʽ
extern char IPAddr[16];//EE��440~455��ASCII����ʽ
extern char PortID[4];//EE��456~459��ASCII����ʽ
extern char TimeAscii[];
extern char TimeAscii[11];//ʱ���ASCII�룬WYYMMDDHHMM
extern char TimeAsciiNew[11];//RS485/GPRS���յ���ʱ���ASCII�룬WYYMMDDHHMM
extern uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//rtc.c���壺���յ������ã�BCD���ʽ������ʱ��
extern char VBtyAscii[];
extern char FTbkljAscii[];
extern char FTgkljAscii[];
extern volatile unsigned char FlagFTgklj;//1:��ȷ��0.1m3��2:��ȷ��1m3
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c�ж���,�����ۼ�����,�洢��EE484-487;����˲ʱ����,��EE�в��洢
extern char FTbkssAscii[];
extern char FTgkssAscii[];
extern char PT1Ascii[];
extern char TTAscii[];//�¶�ASCII��
extern uint16_t NdValue;//����֮�������Ũ��
extern uint8_t BtyValue;//����֮��ĵ�ص�ѹֵ
extern uint8_t SelfStat;//�豸����״̬��bit0-����,bit1-����,bit2-Ƿѹ,bit3-���,bit4-����
extern uint8_t EEArray[];//��Ӧ488��ģ��EE��ַ
extern char TranArray[];//����ǰװ�������,TW2.5
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
//������: Crc16Ascii
//��  ��: ��������ַ�����16λCRC�룬��ת����4�ֽ�ASCII��
//��  ��: num->�ַ������ȣ�Str[]->�ַ���
//��  ��: GPRSCrc16Ascii[4]
//����ֵ: None
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
uint8_t Ascii2Hex(uint8_t AsciiValue)//��ASCII��ת��������
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
	DelayNms(220);//ÿ����λ10ms,��2.2S,powerkey_n 3s 0v 
	GSMUp();

	DelayNms(1500);//ÿ����λ10ms,��15S
}
void turOffGSM(void)
{
	GSMDown();
	DelayNms(220);//powerkey_n 4s 0v 
	GSMUp();
//	DelayNms(300);//powerkey_n 4s 0v
//	EN4VOff();//close 4v
}

void Uart2SendData(volatile uint8_t *Data,uint8_t DataLength)//��ģ�鷢������
{
	uint8_t i=0;

	for(i=0;i<DataLength;i++)
	{
		Tx2Buf[i] = Data[i];
	}
	NbrOfDataToTrans2 = DataLength;//��Ҫ���͵��ֽ���
	Tx2Read=0;
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);//�������ж�
}

void USART2SendStr(volatile char *str)//��ģ�鷢���ַ���
{
	uint8_t IndexStr=0;
	while(str[IndexStr]!='\0'){
		Tx2Buf[IndexStr] = str[IndexStr];
		IndexStr++;
	}
	NbrOfDataToTrans2=IndexStr;
	Tx2Read=0;
	USART2OrderMode=1;//��������Ϊ1
	USART2OrderWait=USART2OrderWaitNum;//��ʱ���ڼ�����100��SysTick�ղ����ظ�Ϊһ�γ�ʱ
	//USART2OrderTimeOut=0;//��ǰ
	USART2OrderErr=0xAA;//δ����ͨѶ��ʱ����
	ProUSART2 = 0xAA;//���������Э��
	Rx2Start = 0xAA;//δ���յ��κλظ�������
	for(IndexStr=0;IndexStr<Rx2BufMax;IndexStr++){//��ս�������
		Rx2Buf[IndexStr]=0;
	}
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);//�������ж�
}
void initGSM(void)
{
	uint8_t CntDly,DestChr;
	char *IndexRx2buf;
	DelayNms(1000);
	//1.ѭ������"AT"�����ģ���Ƿ���,����Ӧ�ظ�"OK"
	for(CntDly=0;CntDly<15;CntDly++){
		USART2SendStr(AT);
		DelayNms(100);//ÿ����λ10ms,��1S//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(ProUSART2 == 0x55)  break ;//�ӵ��ظ�
	}
	if(CntDly>=14){//15�γ�ʱ��˵��GSMģ��δ����
		__nop();
		turOffGSM();//bing gai
		DelayNms(1000);
		EN4VOff();
		DelayNms(500);
		turnOnGSM();
		DelayNms(500);//bing gai over
	}
	if(ProUSART2 == 0x55){//�յ��ظ�����
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){
			__nop();//���Թ۲��Ƿ��յ�OK
		}
	}
	//2.����"ATE0"�رջ���,Ӧ�ظ�"ATE0","OK"
	USART2SendStr(ATE0);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ�OK
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){
			__nop();
		}
	}
	//3.����"AT+CPIN?"��ѯSIM��״̬,����Ӧ�ظ�"+CPIN: READY","OK"
	USART2SendStr(ATCPIN);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//�յ�OK,������
			__nop();
		}else{//�޿�
			__nop();//LEDָʾ�޿�
		}
	}
	//4.����"AT+CREG?"��ѯ��վ����,����Ӧ�ظ�"+CREG: X,Y","OK"  X->MODE  Y->STAT
	USART2SendStr(ATCREG);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		__nop();
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//�յ�OK
			IndexRx2buf=strchr((const char*)Rx2Buf,' ');//char *strchr(const char *,int);
			if(IndexRx2buf!=NULL){
				DestChr=IndexRx2buf[1];//0:��ֹ��ʾ;1:ʹ����ʾ����ʽ:+CREG: X,Y;2:ʹ����ʾ����ʽ:+CREG: X,Y[A,B,C]
			}
			IndexRx2buf=strchr((const char*)Rx2Buf,',');
			if(IndexRx2buf!=NULL){
				DestChr=IndexRx2buf[2];//0:δע��; 1:��ע��; 2:δע�ᣬ����Ѱ��վ;4:δ֪����;5:��ע��,����
			}	
		}//�յ�OK
	}//�յ��ظ�
	//5.����"AT+CSQ"��ѯ�ź�ǿ��,����Ӧ�ظ�"+CSQ: XX, XX","OK"
	USART2SendStr(ATCSQ);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;
		if(strstr((const char*)Rx2Buf,"OK")!='\0'){//�յ�OK
			__nop();
		}//�յ�OK
	}//�յ��ظ�
	//6.����"AT+CGATT?"��ѯGPRS����״̬,����Ӧ�ظ�"+CGATT:X","OK"
	USART2SendStr(ATCGATT);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		//if(strstr((const char*)Rx2Buf,"OK")!='\0'){} //�յ�OK
		IndexRx2buf=strstr((const char*)Rx2Buf,"ATT:");
		if(IndexRx2buf!=NULL){//�յ�+CGATT:X
			DestChr=IndexRx2buf[5];
			if(DestChr=='0'){//δ����
				USART2SendStr(ATCGATT1);//����AT+CGATT=1,ǿ�Ƹ���GPRS
				while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
					__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				}
				if(USART2OrderErr == 0x55){ //��ʱ 
					__nop();
				}
				if(strstr((const char*)Rx2Buf,"OK")!='\0'){//�յ�OK
					__nop();
				} 
			}//δ����
		}//�յ�+CGATT:X
	}
	
}
uint8_t  UDPLink(void)
{
	uint8_t i,NumGPRS;
//1.����"AT+ZPNUM="CMNET","USER","PWD"",����APN����\�û���\����,����Ӧ�ظ�"OK"
//	char ZPNum[]={'A','T','+','Z','P','N','U','M','=','"','C','M','N','E','T','"',
//				      ',','"','U','S','E','R','"',',','"','P','W','D','"',0x0D,'\0'};
	//�������APN�Ĳ���
	for(i=0;i<10;i++) GPRSBuf[i]=ZPNum[i];//1>AT+ZPNUM="
	i=0;NumGPRS=10;
	while(APN[i]!='\0'){//2>װ��APN
		GPRSBuf[NumGPRS++]=APN[i++];//char APN[20];//EE��400~419��ASCII����ʽ
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=','; GPRSBuf[NumGPRS++]='"';
	i=0;
	while(USER[i]!='\0'){//3>װ��USER
		GPRSBuf[NumGPRS++]=USER[i++];//char USER[12];//EE��420~431��ASCII����ʽ
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=','; GPRSBuf[NumGPRS++]='"';
	i=0;
	while(PWD[i]!='\0'){//4>װ��PWD
		GPRSBuf[NumGPRS++]=PWD[i++];//char PWD[8];//EE��432~439��ASCII����ʽ
	}
	GPRSBuf[NumGPRS++]='"'; GPRSBuf[NumGPRS++]=0x0D; GPRSBuf[NumGPRS++]='\0';
	//USART2SendStr(ZPNum);
	USART2SendStr(GPRSBuf);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"OK")){//���յ�OK
			__nop();
		}
	}
	//2.����"AT+ZPPPOPEN,��GPRS����,����Ӧ�ظ�"+ZPPPOPEN:CONNECTED""OK"
	USART2SendStr(ZPPPOpen);
	DelayNms(120);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"CONNECTED")){//���յ�+ZPPPPOPEN:CONNECTED
			__nop();
		}
	}//�յ��ظ�
	//3.����"AT+ZIPSETUPU=1,***.***.***.***,5008,��UDP����,����Ӧ�ظ�"+ZIPSETUPU:CONNECTED" "OK"
	//�������IP��ַ�Ͷ˿ںŵĲ���
	for(i=0;i<14;i++) GPRSBuf[i]=SETUP[i];//1>AT+ZIPSETUPU=1,
	i=0;NumGPRS=14;
	while(IPAddr[i]!='\0'){//2>װ��IPAddr[16]
		GPRSBuf[NumGPRS++]=IPAddr[i++];//char IPAddr[16];//EE��440~455��ASCII����ʽ
	}
	GPRSBuf[NumGPRS++]=',';
	i=0;
	for(i=0;i<4;i++){//3>װ��PortID[4],��4���ֽ�
		GPRSBuf[NumGPRS++]=PortID[i];//char PortID[4];//EE��456~459��ASCII����ʽ
	}
	GPRSBuf[NumGPRS++]=0x0D; GPRSBuf[NumGPRS++]='\0';
	USART2SendStr(GPRSBuf);
	DelayNms(100);//bing gai
	//USART2SendStr(SETUPU);
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"CONNECTED")){//���յ�+ZIPSETUPU:CONNECTED
			__nop();
		}
	}//�յ��ظ�
	//4.����AT+ZTRANSFER=1,1,1000,1000,����Ӧ�ظ�"+ZTRANSFER:1""OK"
	//1:SOCKET1  2:TCP(1:UDP)  1000:1000ms��ʱ,   1000:ÿ֡���1000BYTE
	USART2SendStr(TRANSFER_TCP);
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"TRANSFER:1")){//���յ�+ZTRANSFER:1
			__nop();
		}
	}//�յ��ظ�
	//5.����ATO,����Ӧ�ظ�"Enter into data mode, please input data:""OK"
	USART2SendStr(AT2DATA);
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"OK")){//���յ�+ZTRANSFER:1
			__nop();
			return 1;//�ɹ�����
		}
	}//�յ��ظ�
	return 0;//����ʧ��
}
uint8_t GPRSHisTrans(uint8_t num)//GPRS������ʷ����
{
	uint32_t FlagZhen,DataEE;
	uint16_t IndexEE;
	uint8_t Num;
	uint8_t FlagHis=0;//���ͳɹ���־���ɹ�ʱ�ó�1
	
	Num=num;
	FlagZhen=FlashEERead(Num*100+88);//���֡��־λ
	if((FlagZhen & 0x000000FF)==0x01){//֡��
		for(IndexEE=Num*100;IndexEE<Num*100+88;IndexEE+=4){//����֡����װ��GPRSBuf[],IndexEE�����֡ƫ����
			DataEE=FlashEERead(IndexEE);
			GPRSBuf[IndexEE-Num*100]=(char)(DataEE & 0x000000FF);//���۵ڼ�֡��GPRSBuf[]����0��ʼ
			GPRSBuf[IndexEE-Num*100+1]=(char)((DataEE>>8) & 0x000000FF);
			GPRSBuf[IndexEE-Num*100+2]=(char)((DataEE>>16) & 0x000000FF);
			GPRSBuf[IndexEE-Num*100+3]=(char)((DataEE>>24) & 0x000000FF);
		}
		FlagHis=0;
		USART2SendStr(GPRSBuf);//��һ�η���,��ʷ���ݽ�����һ��
		DelayNms(150);//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(USART2OrderErr == 0x55){ //��ʱ 
			__nop();
		}
		if(ProUSART2 == 0x55){ //�յ��ظ�
			ProUSART2 = 0xAA;__nop();
			if(strstr((const char *)Rx2Buf,"SUCCESS")){//���յ��������ظ�:##SUCCESSCRC@@,CRC��4�ֽ�ASCII��
				FlagHis=1;//���������յ�����
				FlashEEWrite(num*100+88,0);//����֡��־λ
				__nop();
			}
		}
	}else{//֡��
		FlagHis=2;//֡��ʱ����2
	}
	return FlagHis;
}
uint8_t GPRSProTrans(uint8_t Flagtrans)
{
	uint8_t IndexTrans,i;
	uint8_t FlagTrans=0;//���ͳɹ���־���ɹ�ʱ�ó�1
	//uint8_t NumZhen;	
	//uint32_t FlagZhen,DataEE;
	char DeviceIP[18]={'I','P',':','1','2','3','.','2','3','.','4','3','.','3','4','#','#','\0'};
	char *IndexWL,*IndexWT,*IndexW1;
	//1.�ϴ��ɼ��Ĵ���������
	//@@05310000001IP:123.23.43.34##A0000Z
	GPRSBuf[0]='@';GPRSBuf[1]='@';//1>��װ��ͷ
	for(IndexTrans=0;IndexTrans<11;IndexTrans++){//2>����ID,exp:05310000001
		GPRSBuf[IndexTrans+2]=DeviceID[IndexTrans];//��2-12�ֽ�
	}
	for(IndexTrans=0;IndexTrans<17;IndexTrans++){//3>����IP,exp:IP:123.23.43.34##
		GPRSBuf[IndexTrans+13]=DeviceIP[IndexTrans];//��13-29�ֽ�
	}
	for(IndexTrans=0;IndexTrans<6;IndexTrans++){//3>�������� TranArray[]
		GPRSBuf[IndexTrans+30]=TranArray[IndexTrans];//��30-35�ֽ�
	}
	
//	GPRSBuf[36]=0x1A;//4>��װ��β
	GPRSBuf[36]='\0';
	
	FlagTrans=0;
	//USART2SendStr("ABCD");//������
	USART2SendStr(GPRSBuf);//��һ�η���
	DelayNms(50);//bing gai
	while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	if(USART2OrderErr == 0x55){ //��ʱ 
		__nop();
	}
	if(ProUSART2 == 0x55){ //�յ��ظ�
		ProUSART2 = 0xAA;__nop();
		if(strstr((const char *)Rx2Buf,"A") && strchr((const char *)Rx2Buf,'Z')){//���յ��������ظ�:##SUCCESSCRC@@,CRC��4�ֽ�ASCII��
			FlagTrans=1;//���������յ�����
			IndexWL=strstr((const char *)Rx2Buf,"WL");
			if(IndexWL!=NULL){//�����ۻ�����
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
					
				USART2SendStr("%WLOK$");////�ظ�������Ϣ
			}//if(IndexWL!=NULL){//�����ۻ�����
			IndexWT=strstr((const char *)Rx2Buf,"WT");
			if(IndexWT!=NULL){//����ʱ��
				for(i=0;i<11;i++){
					TimeAsciiNew[i]=IndexWT[i+2];
				}
				//�������RTC�����
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
				NewSeconds = 0x30;//������30S��ʱ��������
				ConfigTime(NewHours,NewMinutes,NewSeconds);
				ConfigDate(NewWeek,NewYear,NewMonth,NewDate);
				
				USART2SendStr("%WTOK$");////�ظ�������Ϣ
			}//if(IndexWT!=NULL){//����ʱ��
			IndexW1=strstr((const char *)Rx2Buf,"W1");
			if(IndexW1!=NULL){//����������λ��־
				if(IndexW1[2]=='1')
					FlagFTgklj=1;
				else if(IndexW1[2]=='2')
					FlagFTgklj=2;
				else 
					FlagFTgklj=1;
				EEArray[481]=FlagFTgklj;
				RefreshEE();				
				USART2SendStr("%W1OK$");////�ظ�������Ϣ
			}//if(IndexW1!=NULL){//����������λ��־
			__nop();
		}
	}
	/*
	if(FlagTrans==0){//������δ�ɹ��յ�����
		USART2SendStr(GPRSBuf);//�ط�
		DelayNms(50);//bing gai
		while((ProUSART2 == 0xAA) && (USART2OrderErr != 0x55)){ //�ȴ����������ݣ����ҽ��г�ʱ�ж�
			__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		}
		if(USART2OrderErr == 0x55){ //��ʱ 
			__nop();
		}
		if(ProUSART2 == 0x55){ //�յ��ظ�
			ProUSART2 = 0xAA;__nop();
			if(strstr((const char *)Rx2Buf,"A") && strchr((const char *)Rx2Buf,'Z')){//���յ��������ظ�:##SUCCESSCRC@@,CRC��4�ֽ�ASCII��
				FlagTrans=1;//���������յ�����
				IndexWL=strstr((const char *)Rx2Buf,"WL");
				if(IndexWL!=NULL){//�����ۻ�����
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
					USART2SendStr("%WLOK$");////�ظ�������Ϣ
				}//if(IndexWL!=NULL){//�����ۻ�����
				IndexWT=strstr((const char *)Rx2Buf,"WT");
				if(IndexWT!=NULL){//����ʱ��
					for(i=0;i<11;i++){
						TimeAsciiNew[i]=IndexWT[i+2];
					}
					//�������RTC�����
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
					NewSeconds = 0x30;//������30S��ʱ��������
					ConfigTime(NewHours,NewMinutes,NewSeconds);
					ConfigDate(NewWeek,NewYear,NewMonth,NewDate);			
					USART2SendStr("%WTOK$");////�ظ�������Ϣ
					__nop();	
				}//if(IndexWT!=NULL){//����ʱ��
				IndexW1=strstr((const char *)Rx2Buf,"W1");
				if(IndexW1!=NULL){//����������λ��־
					if(IndexW1[2]=='1')
						FlagFTgklj=1;
					else if(IndexW1[2]=='2')
						FlagFTgklj=2;
					else 
						FlagFTgklj=1;
					EEArray[481]=FlagFTgklj;
					RefreshEE();			
					USART2SendStr("%W1OK$");////�ظ�������Ϣ
				}//if(IndexW1!=NULL){//����������λ��־
			}//if(strstr((const char *)Rx2Buf,"SUCCESS")){
		}//if(ProUSART2 == 0x55){ //�յ��ظ�
	}//if(FlagTrans==0){//������δ�ɹ��յ�����
	//�洢���Ͳ��ɹ���֡
	*/
	if(FlagTrans==0){//��������2����δ�ɹ��յ�����,�洢����δ���ͳɹ�������
		/*
		for(NumZhen=0;NumZhen<4;NumZhen++){
			FlagZhen=FlashEERead(NumZhen*100+88);
			if(FlagZhen){//��ǰ֡��
				if(NumZhen==3){//0-3֡����
					__nop();//�����洢δ��������
				}
			}else{//��ǰ֡��
				for(IndexTrans=0;IndexTrans<88;IndexTrans+=4){//������װ�뵱ǰ֡
					DataEE = (((uint32_t)GPRSBuf[IndexTrans+3]) << 24) & 0xFF000000;
					DataEE |= (((uint32_t)GPRSBuf[IndexTrans+2]) << 16) & 0x00FF0000;
					DataEE |= (((uint32_t)GPRSBuf[IndexTrans+1]) << 8) & 0x0000FF00;
					DataEE |= ((uint32_t)GPRSBuf[IndexTrans]) & 0x000000FF;
					FlashEEWrite(NumZhen*100+IndexTrans,DataEE);
				}
				FlashEEWrite(NumZhen*100+88,1);//��ǰ֡��־�ĳ�1����
				while(NumZhen<4){
					NumZhen++;
					FlashEEWrite(NumZhen*100+88,0);//����ǰ֮֡���֡��־ȫ���ĳɿ�
				}
			}//��ǰ֡��
		}
		*/
		return 0;
	}
	return 1;
}
void ComUDP(int8_t FlagUDP)//�������UDPͨѶ
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
		FlagPro=UDPLink();//�ڶ�������
	
	FlagPro=GPRSProTrans(FlagUDP);
	if(FlagPro==0){
		turOffGSM();
		DelayNms(1500);//powerkey_n 15s 0v
		turnOnGSM();
		initGSM();
		FlagPro=UDPLink();
		FlagPro=GPRSProTrans(FlagUDP);
	}
	//DelayNms(200);//ÿ����λ10ms
	//USART2SendStr(AT2ORDER);
	//DelayNms(200);//ÿ����λ10ms
	//USART2SendStr(ATPwrOff);
	//DelayNms(100);//ÿ����λ10ms
	turOffGSM();
	DelayNms(1000);//powerkey_n 10s 0v
	CloseUSART2();
	Reinit_RCC_HSI();
	Systick_Init();
	Init_GPIO();
}






//********************************USART2����*****************************
void USART2VarInit(void)
{
	Tx2Read = 0;
	CntRx2 = 0; 
	NbrOfDataToTrans2 = 0;
	NbrOfDataToRead2 = 0;
	Rx2Start = 0xAA;//����״̬��־��0x55��ʼ���գ�0xAA�������
	Rx2Read = 0;//����Э��ʱ��ȡ���ݵ�����
	U1DrCopy2 = 0;//USART1������DR�ı���

	ProUSART2 = 0xAA;//0x55�����������,0xAA�������������
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//�ÿ⺯����ֹ�����ж�
	USART2ErrFlag = 0xAA; //�޹���
	USART2OrderWait = 0; //û�����ڵȴ�������
	USART2OrderMode = 1;//��������Ϊ1
	
	USARTErrData = 0;//���մ���ʱ�������ֽ�
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
	USART_ITConfig(USART2, USART_IT_RXNE,ENABLE);//��������ж�
	USART_ITConfig(USART2, USART_IT_TXE,DISABLE);//��ֹ�����ж�
	USART_Cmd(USART2, ENABLE);
}
 
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		if(Rx2Start == 0x55){	//���չ����Ѿ���ʼ�����ֽڲ��ǵ�һ���ֽ�
			DoNothing = 1;//����ռλ�����öϵ���
			CntRec2Nms = 0;	//��3.5�ֽڳ�ʱ������
			//Rx2Buf[CntRx2] = USART_ReceiveData(USART2);
			Rx2Buf[CntRx2] = (uint8_t)(USART2->RDR & (uint8_t)0xFF);
			DoNothing = 1;
			CntRx2++;
			if(CntRx2 > 50){
				CntRx2 = 0;	//���յ�����50���ֽڵ�֡��Э�����޳���50�ֽڵ�֡
				Rx2Start = 0xAA;
			}
		}else{	//���յ�һ���ֽ�
			CntRec2Nms = 0;
			Rx2Start = 0x55;//������ʽ��ʼ
			CntRx2 = 0;
			Rx2Buf[CntRx2++] = (uint8_t)(USART2->RDR & (uint8_t)0xFF);
		}
		//���մ�����
		if(USART_GetFlagStatus(USART2, USART_FLAG_ORE)==SET)//�������
		{USART_ClearFlag(USART2, USART_FLAG_ORE);}
		if(USART_GetFlagStatus(USART2, USART_FLAG_FE)==SET)//֡����
		{USART_ClearFlag(USART2, USART_FLAG_FE);}
   	if(USART_GetFlagStatus(USART2, USART_FLAG_PE)==SET)//У�����
		{USART_ClearFlag(USART2, USART_FLAG_PE);}		
	}//�����жϳ������
	
	if(USART_GetITStatus(USART2,USART_IT_TXE) == ENABLE)
	{
		if(NbrOfDataToTrans2==0)//�������
		{
			//USART1->CR1 &= ~(1 << 7);//�����Ĵ�����ֹ�����ж�
			//GPIOA->ODR &= ~(1<<8); //�����Ĵ�����485���߸ĳɽ���״̬�
			USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//�ÿ⺯����ֹ�����ж�
			//GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);//�ÿ⺯��485���߸ĳɽ���״̬
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

	USART_Cmd(USART2, DISABLE);//�ر�USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,DISABLE );
	//����PA2\PA3�Խ���
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//TX2\RX2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	
	//�ر�MG2639��Դ
	EN4VOff();
	GSMUp();
}
//*------------------------------------------------
//* ������: USART2Trans
//* ��  �ã�USART2���Ϳ��ƺ���
//* ��  ��: String����������������
//* ����ֵ: None
//*------------------------------------------------
void USART2Trans(void)
{
	Tx2Buf[0]=0x55;Tx2Buf[1]=0xAA;Tx2Buf[2]=0x36;Tx2Buf[3]=0x39;
	NbrOfDataToTrans2=4;Tx2Read=0;
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
	//USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//������ɺ����ж��йط����ж�
}
/************************ (C) ��Ȩ �ۿ��� *************************/
