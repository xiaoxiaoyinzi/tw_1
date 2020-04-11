/*****************************************************************************
  *�ļ���: F051test/RS485.c 
  *��@@�ߣ��ۿ��� 
  *�汾�ţ� V1.0
  *��@@�ڣ�2016/02/06
  *��@@����ͨ��UART1����RS485ͨѶ
********************************************************************************/
#include "main.h"

volatile unsigned char  Tx1Buf[40];//USART1�������� 
volatile unsigned char  Rx1Buf[40];//USART1��������
volatile unsigned char  Tx1Read;
volatile unsigned char  CntTx1;//USART1���ͼ�����
volatile unsigned char  CntRx1;//USART1���յ����ֽ��� 
volatile unsigned char  NbrOfDataToTrans1;//��Ҫ���͵��ֽ���
volatile unsigned char  NbrOfDataToRead1;
volatile unsigned char  Rx1Start; //����״̬��־��0x55��ʼ���գ�0xAA�������
volatile unsigned char  Rx1Read; //����Э��ʱ��ȡ���ݵ�����
volatile unsigned char  U1DrCopy1; //USART1������DR�ı���
volatile unsigned char  ProUSART1; //Э����������־��0x55���������0xAA���������
volatile unsigned char USART1ErrFlag; //����1ͨѶ���ϱ�־��0x55����ͨѶ����,0xAAͨѶ����
volatile unsigned char USART1OrderMode; //�������ͣ�
volatile unsigned char USART1OrderWait; //��ʱ����������ģ�鷢��һ�������ó�3������޻ظ���ÿ��SysTick��1
volatile unsigned char USART1OrderTimeOut; //Usart1OrderWait����0ʱ��Usart1OrderTimeOut+1
volatile unsigned char USART1OrderErr; //Usart1OrderTimeOut�ӵ�3ʱ,�ó�0x55��ʾһ��ͨѶ����,����Ϊ0xAA 
volatile unsigned char  USARTErrData; //USART1��USART2�յ��Ĵ�������
uint8_t FlagMode485=0;//0->��ͨģʽ;1->����ģʽ

//����FlashEE����
extern char DeviceID[12];//�豸ʶ���
extern char APN[20];//EE��400~419��ASCII����ʽ
extern char USER[12];//EE��420~431��ASCII����ʽ
extern char PWD[8];//EE��432~439��ASCII����ʽ
extern char IPAddr[16];//EE��440~455��ASCII����ʽ
extern char PortID[4];//EE��456~459��ASCII����ʽ
extern char CircleAD[2];//EE��460~461��ASCII����ʽ
extern char CircleGPRS[2];//EE��464~465��ASCII����ʽ
extern char TimeAscii[11];//ʱ���ASCII�룬WYYMMDDHHMM
extern char TimeAsciiNew[11];//RS485���յ���ʱ���ASCII�룬WYYMMDDHHMM
extern uint8_t NewWeek,NewYear,NewMonth,NewDate,NewHours,NewMinutes,NewSeconds;//rtc.c���壺���յ������ã�BCD���ʽ������ʱ��
extern const int8_t tbl_ascii[];//uart_GSM.c�ж���
extern char VBtyAscii[];//ADC_Calcu.c�ж���,��ص�ѹASCII ����"3.67"
extern char FTgkssAscii[];//ADC_Calcu.c�ж���
extern char FTgkljAscii[];//ADC_Calcu.c�ж���
extern char PT1Ascii[];//ADC_Calcu.c�ж���
extern char TTAscii[];//ADC_Calcu.c�ж���
extern volatile unsigned char FlagFTgklj;//1:��ȷ��0.1m3��2:��ȷ��1m3
extern volatile uint32_t FTgklj,FTgkss;//ADC_Calcu.c�ж���,�����ۼ�����,�洢��EE484-487;����˲ʱ����,��EE�в��洢
extern uint8_t EEArray[];//��Ӧ488��ģ��EE��ַ
//*----------------------------------------------------------
//������: Rs485
//��  ��: ���RS485���÷�ʽ,�����շ�����������
//��  ��: None
//��  ��: None
//����ֵ: None
//*----------------------------------------------------------
void Rs485(void)//
{
	uint8_t i;
	//uint32_t EEData;
	char ReWrite[8]={'%','W','E','O','K','$',0x0A,'\0'};
	//*
	if(SetEn()&&(FlagMode485==0)){//���뿪��������ģʽ,���������ڲ�������ģʽ
		FlagMode485=1;//�޸�Ϊ����ģʽ
		LEDSetOn();
		USART1Init();
		USART1VarInit();
	}else if(SetDis()){
		FlagMode485=0;//�޸�Ϊ��ͨģʽ
		LEDSetOff();
		CloseUSART1();
		FX485IN();
		DIS485();
	}
	//*/
	if(ProUSART1 == 0x55){//0x55�����������,0xAA�������������
		ProUSART1 = 0xAA;
		if((Rx1Buf[0]=='%')&&(Rx1Buf[1]=='W')){//д���ò���
			switch(Rx1Buf[2]){
				case 'E'://1.����DeviceID
					for(i=0;Rx1Buf[i+3]!='$';i++){
						DeviceID[i]=Rx1Buf[i+3];
					}
					DeviceID[i]='\0';//��'\0'����
					for(i=0;i<12;i++){
						EEArray[468+i]=DeviceID[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='E';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'A'://2.����APN
					for(i=0;Rx1Buf[i+3]!='$';i++){
						APN[i]=Rx1Buf[i+3];
					}
					APN[i]='\0';//��'\0'����
					for(i=0;i<20;i++){
						EEArray[400+i]=APN[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='A';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'U'://3.����USER
					for(i=0;Rx1Buf[i+3]!='$';i++){
						USER[i]=Rx1Buf[i+3];
					}
					USER[i]='\0';//��'\0'����
					for(i=0;i<12;i++){
						EEArray[420+i]=USER[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='U';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'P'://4.����PWD
					for(i=0;Rx1Buf[i+3]!='$';i++){
						PWD[i]=Rx1Buf[i+3];
					}
					PWD[i]='\0';//��'\0'����
					for(i=0;i<8;i++){
						EEArray[432+i]=PWD[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='P';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'I'://5.����IP
					for(i=0;Rx1Buf[i+3]!='$';i++){
						IPAddr[i]=Rx1Buf[i+3];
					}
					IPAddr[i]='\0';//��'\0'����
					for(i=0;i<16;i++){
						EEArray[440+i]=IPAddr[i];
					}
					RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='I';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'O'://6.����PortID,4���ֽ�
					for(i=0;Rx1Buf[i+3]!='$';i++){
						PortID[i]=Rx1Buf[i+3];
					}
						EEArray[459] = PortID[3];
						EEArray[458] = PortID[2];
						EEArray[457] = PortID[1];
						EEArray[456] = PortID[0];
						RefreshEE();
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='O';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'D'://7.����AD�ɼ�����,2���ֽ�
					CircleAD[0]=Rx1Buf[3];
					CircleAD[1]=Rx1Buf[4];
						EEArray[461] = CircleAD[1];
						EEArray[460] = CircleAD[0];
						RefreshEE();
						for(i=0;i<8;i++){//�ظ�������Ϣ
							Tx1Buf[i]=ReWrite[i];
						}
						Tx1Buf[2]='D';
						NbrOfDataToTrans1=8;
						Tx1Read=0;
						FX485OUT();
						USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'G'://8.����GPRS����,2���ֽ�
					CircleGPRS[0]=Rx1Buf[3];
					CircleGPRS[1]=Rx1Buf[4];
						EEArray[465] = CircleGPRS[1];
						EEArray[464] = CircleGPRS[0];
						RefreshEE();
						for(i=0;i<8;i++){//�ظ�������Ϣ
							Tx1Buf[i]=ReWrite[i];
						}
						Tx1Buf[2]='G';
						NbrOfDataToTrans1=8;
						Tx1Read=0;
						FX485OUT();
						USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'L'://9.�����ۻ�����
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
					
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='L';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'T'://10.����RTC
					for(i=0;i<11;i++){
						TimeAsciiNew[i]=Rx1Buf[i+3];
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
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='T';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
				break;
				case '1'://11.����������λ��־
					if(Rx1Buf[3]=='1'){//0.1m3
						FlagFTgklj=1;
					}else if(Rx1Buf[3]=='2'){//1m3
						FlagFTgklj=2;
					}
					EEArray[481]=FlagFTgklj;
					RefreshEE();
					
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='1';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
				break;
				case '2'://12.�궨
					for(i=0;i<8;i++){//�ظ�������Ϣ
						Tx1Buf[i]=ReWrite[i];
					}
					Tx1Buf[2]='2';
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					
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
		}else if((Rx1Buf[0]=='%')&&(Rx1Buf[1]=='R')){//�����ò���
			switch(Rx1Buf[2]){
				case 'E'://1.��DeviceID
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='E';//֡ͷ
					for(i=0;DeviceID[i]!='\0';i++){
						Tx1Buf[i+3]=DeviceID[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//֡β
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'A'://2.��APN
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='A';//֡ͷ
					for(i=0;APN[i]!='\0';i++){
						Tx1Buf[i+3]=APN[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//֡β
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'U'://3.��USER
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='U';//֡ͷ
					for(i=0;USER[i]!='\0';i++){
						Tx1Buf[i+3]=USER[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//֡β
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'P'://4.��PWD
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='P';//֡ͷ
					for(i=0;PWD[i]!='\0';i++){
						Tx1Buf[i+3]=PWD[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//֡β
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'I'://5.��IP
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='I';//֡ͷ
					for(i=0;IPAddr[i]!='\0';i++){
						Tx1Buf[i+3]=IPAddr[i];
					}
					Tx1Buf[i+3]='$';Tx1Buf[i+4]=0x0A;Tx1Buf[i+5]='\0';//֡β
					NbrOfDataToTrans1=i+6;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'O'://6.��PortID,4���ֽ�
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='O';//֡ͷ
					Tx1Buf[3]=PortID[0];Tx1Buf[4]=PortID[1];Tx1Buf[5]=PortID[2];Tx1Buf[6]=PortID[3];
					Tx1Buf[7]='$';Tx1Buf[8]=0x0A;Tx1Buf[9]='\0';//֡β
					NbrOfDataToTrans1=10;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'D'://7.��AD�ɼ�����,2���ֽ�
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='D';//֡ͷ
					Tx1Buf[3]=CircleAD[0];Tx1Buf[4]=CircleAD[1];
					Tx1Buf[5]='$';Tx1Buf[6]=0x0A;Tx1Buf[7]='\0';//֡β
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'G'://8.��GPRS����,2���ֽ�
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='G';//֡ͷ
					Tx1Buf[3]=CircleGPRS[0];Tx1Buf[4]=CircleGPRS[1];
					Tx1Buf[5]='$';Tx1Buf[6]=0x0A;Tx1Buf[7]='\0';//֡β
					NbrOfDataToTrans1=8;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'L'://9.���ۼ�����,8���ֽ�
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='L';//֡ͷ
					for(i=0;i<9;i++){
						Tx1Buf[i+3]=FTgkljAscii[i+1];
					}
					Tx1Buf[12]='$';Tx1Buf[13]=0x0A;Tx1Buf[14]='\0';//֡β
					NbrOfDataToTrans1=15;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case 'T'://10.����ǰʱ��
					GetTimeAscii();
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='T';//֡ͷ
					for(i=0;i<11;i++){
						Tx1Buf[i+3]=TimeAscii[i];
					}
					Tx1Buf[14]='$';Tx1Buf[15]=0x0A;Tx1Buf[16]='\0';//֡β
					NbrOfDataToTrans1=17;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				case '1'://11.��������λ��־
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='1';//֡ͷ
					if(FlagFTgklj==1)
						Tx1Buf[3]='1';
					else if(FlagFTgklj==2)
						Tx1Buf[3]='2';
					else
						Tx1Buf[3]='1';
					Tx1Buf[4]='$';Tx1Buf[5]=0x0A;Tx1Buf[6]='\0';//֡β
					NbrOfDataToTrans1=7;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
					case '2'://12.����������
					Tx1Buf[0]='%';Tx1Buf[1]='R';Tx1Buf[2]='2';//1>֡ͷ
					Tx1Buf[3]=VBtyAscii[0];Tx1Buf[4]=VBtyAscii[1];Tx1Buf[5]=VBtyAscii[2];Tx1Buf[6]=VBtyAscii[3];//2>��ص�ѹ
					for(i=0;i<8;i++) Tx1Buf[7+i]=FTgkljAscii[1+i];//3>�����ۻ�����7~14
					for(i=0;i<8;i++) Tx1Buf[15+i]=FTgkssAscii[1+i];//4>����˲ʱ����15~22
					Tx1Buf[23]=PT1Ascii[1];Tx1Buf[24]=PT1Ascii[2];Tx1Buf[25]=PT1Ascii[3];Tx1Buf[26]=PT1Ascii[4];
					Tx1Buf[27]=TTAscii[1];Tx1Buf[28]=TTAscii[2];
					Tx1Buf[29]='$';Tx1Buf[30]=0x0A;Tx1Buf[31]='\0';//֡β
					NbrOfDataToTrans1=32;
					Tx1Read=0;
					FX485OUT();
					USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//ʹ��USART1�����ж�
					break;
				default:
					__nop();
					break;
			}//switch(Rx1Buf[2])
		}else{//�޷�ʶ�������
			__nop();
		}		
	}//if(ProUSART1 == 0x55)
}



//*------------------------------------------------
//* ������: Usart1VarInit
//* ��  �ã���ʼ��USART1�ĸ�������
//* ��  ��: None
//* ����ֵ: None
//*------------------------------------------------
void USART1VarInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; //����ṹ��
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
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;//��
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;//
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;//
	GPIO_Init(GPIOA, &GPIO_InitStruct); //
	EN485();//Ϊ485��·����
	
	Tx1Read = 0;
	CntRx1 = 0; 
	NbrOfDataToTrans1 = 0;
	NbrOfDataToRead1 = 0;
	Rx1Start = 0xAA;//����״̬��־��0x55��ʼ���գ�0xAA�������
	Rx1Read = 0;//����Э��ʱ��ȡ���ݵ�����
	U1DrCopy1 = 0;//USART1������DR�ı���
	USARTErrData = 0;//���մ���ʱ�������ֽ�
	ProUSART1 = 0xAA;//0x55�����������,0xAA�������������
	//USART1->CR1 &= ~(1 << 7);//����TXIE
	//GPIOA->ODR = (GPIOA->ODR & ~(1<<8));//485����ģʽ
	USART1ErrFlag = 0xAA; //�޹���
	USART1OrderWait = 0; //û�����ڵȴ�������
	USART1OrderMode = 1;//��������Ϊ1
}

void USART1Init(void)
{
	//�����ڲ�����
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef      NVIC_Init_Struct;//�����жϳ�ʼ��
	
	//��ʼ��USART1��ص�PA9,PA10
//	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);//���ù���ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);//���ù���ΪUSART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//��ʼ��������ص��ж�
	NVIC_Init_Struct.NVIC_IRQChannel = USART1_IRQn;//stm32f0xx.h�ж����ж�����:USART1_IRQn = 27,  
	NVIC_Init_Struct.NVIC_IRQChannelPriority = 0x01;
	NVIC_Init_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Init_Struct);
	//��ʼ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE ); //usart1����ʹ��
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 9600;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//
	USART_Init(USART1, &USART_InitStructure); //��ʼ������
	USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_TXE,DISABLE);//��ֹ�����ж�
	USART_Cmd(USART1, ENABLE);//
}

void CloseUSART1(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	FX485IN();
	//CloseUsart1();
	USART_Cmd(USART1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE ); //usart1����ʹ��
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
void USART1_IRQHandler(void)//USART1�����жϺ���
{
	uint8_t linshi;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == !RESET)
	{
		if((uint8_t)(USART1->RDR & (uint8_t)0xFF)=='%'){//ÿ֡�ĵ�һ���ֽ�
			Rx1Start=0x55;
			CntRx1=0;
			Rx1Buf[CntRx1++]=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
		}else if(Rx1Start==0x55){//��ǰ֡�����Ѿ���ʼ
			Rx1Buf[CntRx1]=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
			if(Rx1Buf[CntRx1]=='$'){//ÿ֡�����һ���ֽ�
				Rx1Start=0xAA;
				ProUSART1=0x55;
			}
			CntRx1++;
		}else{
			linshi=(uint8_t)(USART1->RDR & (uint8_t)0xFF);
			if(linshi)//����ռλ�����öϵ���
				__nop();
		}
		//���մ�����
		if(USART_GetFlagStatus(USART1, USART_FLAG_ORE)==SET)//�������
		{USART_ClearFlag(USART1, USART_FLAG_ORE);}
		if(USART_GetFlagStatus(USART1, USART_FLAG_FE)==SET)//֡����
		{USART_ClearFlag(USART1, USART_FLAG_FE);}
   	if(USART_GetFlagStatus(USART1, USART_FLAG_PE)==SET)//У�����
		{USART_ClearFlag(USART1, USART_FLAG_PE);}		
	}//if(USART_GetITStatus(USART1,USART_IT_RXNE) == ENABLE)
       //
	if(USART_GetITStatus(USART1,USART_IT_TXE) == ENABLE)
	{
		if(NbrOfDataToTrans1==0)//�������
		//if(Tx1Buf[Tx1Read]=='\0')//�������
		{
			//USART1->CR1 &= ~(1 << 7);//�����Ĵ�����ֹ�����ж�
			FX485IN();; //�����Ĵ�����485���߸ĳɽ���״̬
			USART_ITConfig(USART1,USART_IT_TXE,DISABLE);//�ÿ⺯����ֹ�����ж�
			//GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);//�ÿ⺯��485���߸ĳɽ���״̬
		}else{
			//USART1->TDR = (uint16_t)Tx1Buf[Tx1Read++] & (uint16_t)0x01FF;
			USART1->TDR = Tx1Buf[Tx1Read++];
			//USART_SendData(USART1, (uint16_t)Tx1Buf[Tx1Read++]);
			NbrOfDataToTrans1--;
		}
	}
}

//*------------------------------------------------
//* ������: USART1Trans
//* ��  �ã�USART1���Ϳ��ƺ���
//* ��  ��: String����������������
//* ����ֵ: None
//*------------------------------------------------
void USART1Trans(void)
{
	Tx1Buf[0]=0x55;Tx1Buf[1]=0xAA;Tx1Buf[2]=0x36;Tx1Buf[3]=0x39;//���� 55 AA 36 39����USART1
	NbrOfDataToTrans1=4;Tx1Read=0;
	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);//������ɺ����ж��йط����ж�
}
/************************ (C) ��Ȩ �ۿ��� *************************/
