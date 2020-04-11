/*****************************************************************************
  *文件名: F051test/RS485.c 
  *作@@者：邵开春 
  *版本号： V1.0
  *日@@期：2016/02/06
  *描@@述：通过UART1进行RS485通讯
********************************************************************************/
#include "main.h"

volatile unsigned char  Tx1Buf[40];//USART1发送数组 
volatile unsigned char  Rx1Buf[40];//USART1接收数组
volatile unsigned char  Tx1Read;
volatile unsigned char  CntTx1;//USART1发送计数器
volatile unsigned char  CntRx1;//USART1接收到的字节数 
volatile unsigned char  NbrOfDataToTrans1;//将要发送的字节数
volatile unsigned char  NbrOfDataToRead1;
volatile unsigned char  Rx1Start; //接收状态标志，0x55开始接收，0xAA接收完成
volatile unsigned char  Rx1Read; //解析协议时读取数据的索引
volatile unsigned char  U1DrCopy1; //USART1接收器DR的备份
volatile unsigned char  ProUSART1; //协议解析允许标志，0x55允许解析，0xAA不允许解析
volatile unsigned char USART1ErrFlag; //串口1通讯故障标志：0x55发生通讯故障,0xAA通讯正常
volatile unsigned char USART1OrderMode; //命令类型：
volatile unsigned char USART1OrderWait; //超时计数器，向模块发送一次命令置成3，如果无回复，每个SysTick减1
volatile unsigned char USART1OrderTimeOut; //Usart1OrderWait减到0时，Usart1OrderTimeOut+1
volatile unsigned char USART1OrderErr; //Usart1OrderTimeOut加到3时,置成0x55表示一次通讯故障,正常为0xAA 
volatile unsigned char  USARTErrData; //USART1或USART2收到的错误数据
uint8_t FlagMode485=0;//0->普通模式;1->配置模式

//定义FlashEE数据
extern char DeviceID[12];//设备识别号
extern char APN[20];//EE中400~419，ASCII码型式
extern char USER[12];//EE中420~431，ASCII码型式
extern char PWD[8];//EE中432~439，ASCII码型式
extern char IPAddr[16];//EE中440~455，ASCII码型式
extern char PortID[4];//EE中456~459，ASCII码型式
extern char CircleAD[2];//EE中460~461，ASCII码型式
extern char CircleGPRS[2];//EE中464~465，ASCII码型式
extern char TimeAscii[11];//时间的ASCII码，WYYMMDDHHMM
extern char TimeAsciiNew[11];//RS485接收到的时间的ASCII码，WYYMMDDHHMM
extern uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//rtc.c定义：接收到的设置，BCD码格式的日期时间
extern const int8_t tbl_ascii[];//uart_GSM.c中定义
extern char VBtyAscii[];//ADC_Calcu.c中定义,电池电压ASCII 例如"3.67"
extern char FTgkssAscii[];//ADC_Calcu.c中定义
extern char FTgkljAscii[];//ADC_Calcu.c中定义
extern char PT1Ascii[];//ADC_Calcu.c中定义
extern char TTAscii[];//ADC_Calcu.c中定义
extern volatile unsigned char FlagFTgklj;//1:精确到0.1m3，2:精确到1m3
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c中定义,工况累计流量,存储在EE484-487;工况瞬时流量,在EE中不存储
extern uint8_t EEArray[];//对应488个模拟EE地址
//*----------------------------------------------------------
//函数名: Rs485
//功  能: 检测RS485配置方式,进行收发、解析数据
//输  入: None
//输  出: None
//返回值: None
//*----------------------------------------------------------
void Rs485(void)//
{
	uint8_t i;
	//uint32_t EEData;
	char ReWrite[8]={'%','W','E','O','K','$',0x0A,'\0'};
	//*
	if(SetEn()&&(FlagMode485==0)){//拨码开关在设置模式,并且上周期不在设置模式
		FlagMode485=1;//修改为设置模式
		LEDSetOn();
		USART1Init();
		USART1VarInit();
	}else if(SetDis()){
		FlagMode485=0;//修改为普通模式
		LEDSetOff();
		CloseUSART1();
		FX485IN();
		DIS485();
	}
	//*/
	if(ProUSART1 == 0x55){//0x55允许解析数据,0xAA不允许解析数据
		ProUSART1 = 0xAA;
		if((Rx1Buf[0]=='%')&&(Rx1Buf[1]=='W')){//写配置参数
			switch(Rx1Buf[2]){
				case 'E'://1.配置DeviceID
					for(i=0;Rx1Buf[i+3]!='$';i++){
						DeviceID[i]=Rx1Buf[i+3];
					}
					DeviceID[i]='\0';//以'\0'结束
					for(i=0;i<12;i++){
						EEArray[468+i]=DeviceID[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='E';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'A'://2.配置APN
					for(i=0;Rx1Buf[i+3]!='$';i++){
						APN[i]=Rx1Buf[i+3];
					}
					APN[i]='\0';//以'\0'结束
					for(i=0;i<20;i++){
						EEArray[400+i]=APN[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='A';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'U'://3.配置USER
					for(i=0;Rx1Buf[i+3]!='$';i++){
						USER[i]=Rx1Buf[i+3];
					}
					USER[i]='\0';//以'\0'结束
					for(i=0;i<12;i++){
						EEArray[420+i]=USER[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='U';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'P'://4.配置PWD
					for(i=0;Rx1Buf[i+3]!='$';i++){
						PWD[i]=Rx1Buf[i+3];
					}
					PWD[i]='\0';//以'\0'结束
					for(i=0;i<8;i++){
						EEArray[432+i]=PWD[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='P';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'I'://5.配置IP
					for(i=0;Rx1Buf[i+3]!='$';i++){
						IPAddr[i]=Rx1Buf[i+3];
					}
					IPAddr[i]='\0';//以'\0'结束
					for(i=0;i<16;i++){
						EEArray[440+i]=IPAddr[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='I';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'O'://6.配置PortID,4个字节
					for(i=0;Rx1Buf[i+3]!='$';i++){
						PortID[i]=Rx1Buf[i+3];
					}
						EEArray[459] = PortID[3];
						EEArray[458] = PortID[2];
						EEArray[457] = PortID[1];
						EEArray[456] = PortID[0];
						RefreshEE();
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='O';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'D'://7.配置AD采集周期,2个字节
					CircleAD[0]=Rx1Buf[3];
					CircleAD[1]=Rx1Buf[4];
						EEArray[461] = CircleAD[1];
						EEArray[460] = CircleAD[0];
						RefreshEE();
						for(i=0;i<8;i++){//回复配置信息
							Tx1Buf[i]=ReWrite[i];
						}
						Tx1Buf[2]='D';
						NbrOfDataToTrans1=8;
						Tx1Read=0;
						FX485OUT();
						USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'G'://8.配置GPRS周期,2个字节
					CircleGPRS[0]=Rx1Buf[3];
					CircleGPRS[1]=Rx1Buf[4];
						EEArray[465] = CircleGPRS[1];
						EEArray[464] = CircleGPRS[0];
						RefreshEE();
						for(i=0;i<8;i++){//回复配置信息
							Tx1Buf[i]=ReWrite[i];
						}
						Tx1Buf[2]='G';
						NbrOfDataToTrans1=8;
						Tx1Read=0;
						FX485OUT();
						USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'L'://9.配置累积流量
					FTgkljAscii[1] = Rx1Buf[3];
					FTgkljAscii[2] = Rx1Buf[4];
					FTgkljAscii[3] = Rx1Buf[5];
					FTgkljAscii[4] = Rx1Buf[6];
					FTgkljAscii[5] = Rx1Buf[7];
					FTgkljAscii[6] = Rx1Buf[8];
					FTgkljAscii[7] = Rx1Buf[9];
					FTgkljAscii[8] = Rx1Buf[10];
					FTgklj = 0xF0000000 & (Ascii2Hex(Rx1Buf[3])<<28);
					FTgklj |=  0x0F000000 & (Ascii2Hex(Rx1Buf[4])<<24);
					FTgklj |=  0x00F00000 & (Ascii2Hex(Rx1Buf[5])<<20);
					FTgklj |=  0x000F0000 & (Ascii2Hex(Rx1Buf[6])<<16);
					FTgklj |=  0x0000F000 & (Ascii2Hex(Rx1Buf[7])<<12);
					FTgklj |=  0x00000F00 & (Ascii2Hex(Rx1Buf[8])<<8);
					FTgklj |=  0x000000F0 & (Ascii2Hex(Rx1Buf[9])<<4);
					FTgklj |=  0x0000000F & Ascii2Hex(Rx1Buf[10]);
					EEArray[484]=FTgklj & 0xFF;EEArray[485]=(FTgklj>>8) & 0xFF;
					EEArray[486]=(FTgklj>>16) & 0xFF;
					RefreshEE();
					
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='L';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'T'://10.配置RTC
					for(i=0;i<11;i++){
						TimeAsciiNew[i]=Rx1Buf[i+3];
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
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='T';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
				break;
				case '1'://11.配置流量单位标志
					if(Rx1Buf[3]=='1'){//0.1m3
						FlagFTgklj=1;
					}else if(Rx1Buf[3]=='2'){//1m3
						FlagFTgklj=2;
					}
					EEArray[481]=FlagFTgklj;
					RefreshEE();
					
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='1';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
				break;
				case '2'://12.标定
					for(i=0;i<8;i++){//回复配置信息
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='2';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					
					if((Rx1Buf[3]=='1')&&(Rx1Buf[4]=='0')){//10%LEL
						NdBiaoding(10);
					}else if((Rx1Buf[3]=='5')&&(Rx1Buf[4]=='0')){//1m3
						NdBiaoding(50);
					}
				break;
				default:
					__nop();
					break;
			}//switch(Rx1Buf[2])
		}else if((Rx1Buf[0]=='%')&&(Rx1Buf[1]=='R')){//读配置参数
			switch(Rx1Buf[2]){
				case 'E'://1.读DeviceID
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='E';//帧头
					for(i=0;DeviceID[i]!='\0';i++){
						Tx1Buf[i+3]=DeviceID[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//帧尾
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'A'://2.读APN
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='A';//帧头
					for(i=0;APN[i]!='\0';i++){
						Tx1Buf[i+3]=APN[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//帧尾
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'U'://3.读USER
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='U';//帧头
					for(i=0;USER[i]!='\0';i++){
						Tx1Buf[i+3]=USER[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//帧尾
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'P'://4.读PWD
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='P';//帧头
					for(i=0;PWD[i]!='\0';i++){
						Tx1Buf[i+3]=PWD[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//帧尾
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'I'://5.读IP
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='I';//帧头
					for(i=0;IPAddr[i]!='\0';i++){
						Tx1Buf[i+3]=IPAddr[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//帧尾
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'O'://6.读PortID,4个字节
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='O';//帧头
					Tx1Buf[3]=PortID[0];Tx1Buf[4]=PortID[1];Tx1Buf[5]=PortID[2];Tx1Buf[6]=PortID[3];
					Tx1Buf[7]='$';Tx1Buf[8]=0x0A;Tx1Buf[9]='\0';//帧尾
					NbrOfDataToTrans1=10;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'D'://7.读AD采集周期,2个字节
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='D';//帧头
					Tx1Buf[3]=CircleAD[0];Tx1Buf[4]=CircleAD[1];
					Tx1Buf[5]='$';Tx1Buf[6]=0x0A;Tx1Buf[7]='\0';//帧尾
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'G'://8.读GPRS周期,2个字节
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='G';//帧头
					Tx1Buf[3]=CircleGPRS[0];Tx1Buf[4]=CircleGPRS[1];
					Tx1Buf[5]='$';Tx1Buf[6]=0x0A;Tx1Buf[7]='\0';//帧尾
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'L'://9.读累计流量,8个字节
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='L';//帧头
					for(i=0;i<9;i++){
						Tx1Buf[i+3]=FTgkljAscii[i+1];
					}
					Tx1Buf[12]='$';Tx1Buf[13]=0x0A;Tx1Buf[14]='\0';//帧尾
					NbrOfDataToTrans1=15;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case 'T'://10.读当前时间
					GetTimeAscii();
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='T';//帧头
					for(i=0;i<11;i++){
						Tx1Buf[i+3]=TimeAscii[i];
					}
					Tx1Buf[14]='$';Tx1Buf[15]=0x0A;Tx1Buf[16]='\0';//帧尾
					NbrOfDataToTrans1=17;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				case '1'://11.读流量单位标志
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='1';//帧头
					if(FlagFTgklj==1)
						Tx1Buf[3]='1';
					else if(FlagFTgklj==2)
						Tx1Buf[3]='2';
					else
						Tx1Buf[3]='1';
					Tx1Buf[4]='$';Tx1Buf[5]=0x0A;Tx1Buf[6]='\0';//帧尾
					NbrOfDataToTrans1=7;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
					case '2'://12.读测量参数
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='2';//1>帧头
					Tx1Buf[3]=VBtyAscii[0];Tx1Buf[4]=VBtyAscii[1];Tx1Buf[5]=VBtyAscii[2];Tx1Buf[6]=VBtyAscii[3];//2>电池电压
					for(i=0;i<8;i++) Tx1Buf[7+i]=FTgkljAscii[1+i];//3>工况累积量，7~14
					for(i=0;i<8;i++) Tx1Buf[15+i]=FTgkssAscii[1+i];//4>工况瞬时量，15~22
					Tx1Buf[23]=PT1Ascii[1];Tx1Buf[24]=PT1Ascii[2];Tx1Buf[25]=PT1Ascii[3];Tx1Buf[26]=PT1Ascii[4];
					Tx1Buf[27]=TTAscii[1];Tx1Buf[28]=TTAscii[2];
					Tx1Buf[29]='$';Tx1Buf[30]=0x0A;Tx1Buf[31]='\0';//帧尾
					NbrOfDataToTrans1=32;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能USART1发送中断
					break;
				default:
					__nop();
					break;
			}//switch(Rx1Buf[2])
		}else{//无法识别的命令
			__nop();
		}		
	}//if(ProUSART1 == 0x55)
}



//*------------------------------------------------
//* 函数名: Usart1VarInit
//* 作  用：初始化USART1的辅助变量
//* 参  数: None
//* 返回值: None
//*------------------------------------------------
void USART1VarInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; //定义结构体
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;//SET
	//GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	//GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	//GPIO_Init(GPIOA, &GPIO_InitStruct); //
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;//FX485
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	FX485IN();
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;//空
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	EN485();//为485电路供电
	
	Tx1Read = 0;
	CntRx1 = 0; 
	NbrOfDataToTrans1 = 0;
	NbrOfDataToRead1 = 0;
	Rx1Start = 0xAA;//接收状态标志，0x55开始接收，0xAA接收完成
	Rx1Read = 0;//解析协议时读取数据的索引
	U1DrCopy1 = 0;//USART1接收器DR的备份
	USARTErrData = 0;//接收错误时的数据字节
	ProUSART1 = 0xAA;//0x55允许解析数据,0xAA不允许解析数据
	//USART1->CR1 &= ~(1 << 7);//清零TXIE
	//GPIOA->ODR = (GPIOA->ODR & ~(1<<8));//485接收模式
	USART1ErrFlag = 0xAA; //无故障
	USART1OrderWait = 0; //没有正在等待的命令
	USART1OrderMode = 1;//命令类型为1
}

void USART1Init(void)
{
	//定义内部变量
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef      NVIC_Init_Struct;//串口中断初始化
	
	//初始化USART1相关的PA9,PA10
//	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);//复用功能为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);//复用功能为USART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//初始化串口相关的中断
	NVIC_Init_Struct.NVIC_IRQChannel = USART1_IRQn;//stm32f0xx.h中定义中断向量:USART1_IRQn = 27,  
	NVIC_Init_Struct.NVIC_IRQChannelPriority = 0x01;
	NVIC_Init_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Init_Struct);
	//初始化串口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE ); //usart1总线使能
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 9600;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//
	USART_Init(USART1, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);//允许接收中断
	USART_ITConfig(USART1, USART_IT_TXE,DISABLE);//禁止发送中断
	USART_Cmd(USART1, ENABLE);//
}

void CloseUSART1(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	FX485IN();
	//CloseUsart1();
	USART_Cmd(USART1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE ); //usart1总线使能
	/*
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;//TX1\RX1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);
	GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	*/
//*
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;//TX1\RX1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);
	GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	//*/
}
void USART1_IRQHandler(void)//USART1串口中断函数
{
	uint8_t linshi;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == !RESET)
	{
		if((uint8_t)(USART1->RDR & (uint8_t)0xFF)=='%'){//每帧的第一个字节
			Rx1Start=0x55;
			CntRx1=0;
			Rx1Buf[CntRx1++]=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
		}else if(Rx1Start==0x55){//当前帧接收已经开始
			Rx1Buf[CntRx1]=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
			if(Rx1Buf[CntRx1]=='$'){//每帧的最后一个字节
				Rx1Start=0xAA;
				ProUSART1=0x55;
			}
			CntRx1++;
		}else{
			linshi=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
			if(linshi)//调试占位，设置断点用
				__nop();
		}
		//接收错误处理
		if(USART_GetFlagStatus(USART1, USART_FLAG_ORE)==SET)//溢出错误
		{USART_ClearFlag(USART1, USART_FLAG_ORE);}
		if(USART_GetFlagStatus(USART1, USART_FLAG_FE)==SET)//帧错误
		{USART_ClearFlag(USART1, USART_FLAG_FE);}
   	if(USART_GetFlagStatus(USART1, USART_FLAG_PE)==SET)//校验错误
		{USART_ClearFlag(USART1, USART_FLAG_PE);}		
	}//if(USART_GetITStatus(USART1,USART_IT_RXNE) == ENABLE)
       //
	if(USART_GetITStatus(USART1,USART_IT_TXE) == ENABLE)
	{
		if(NbrOfDataToTrans1==0)//发送完成
		//if(Tx1Buf[Tx1Read]=='\0')//发送完成
		{
			//USART1->CR1 &= ~(1 << 7);//操作寄存器禁止发送中断
			FX485IN();; //操作寄存器将485总线改成接收状态
			USART_ITConfig(USART1,USART_IT_TXE,DISABLE);//用库函数禁止发送中断
			//GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);//用库函数485总线改成接收状态
		}else{
			//USART1->TDR = (uint16_t)Tx1Buf[Tx1Read++] & (uint16_t)0x01FF;
			USART1->TDR = Tx1Buf[Tx1Read++];
			//USART_SendData(USART1, (uint16_t)Tx1Buf[Tx1Read++]);
			NbrOfDataToTrans1--;
		}
	}
}

//*------------------------------------------------
//* 函数名: USART1Trans
//* 作  用：USART1发送控制函数
//* 参  数: String，发送数据数组名
//* 返回值: None
//*------------------------------------------------
void USART1Trans(void)
{
	Tx1Buf[0]=0x55;Tx1Buf[1]=0xAA;Tx1Buf[2]=0x36;Tx1Buf[3]=0x39;//发送 55 AA 36 39测试USART1
	NbrOfDataToTrans1=4;Tx1Read=0;
	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);//发送完成后在中断中关发送中断
}
/************************ (C) 版权 邵开春 *************************/
