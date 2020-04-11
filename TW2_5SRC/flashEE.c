#include "main.h"

// ����˽��ö������
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
// ˽�е�define 
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   //ÿҳΪ1K BYTE
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800F000)   //оƬFlash����ʼ��ַΪ 0x08000000
#define FLASH_USER_END_ADDR     ((uint32_t)0x0800FFFF)   //����data�洢�ĵ�ַ���䣺0x0800F000~0x0800FFFF,��4K BYTE
#define DATA_32                 ((uint32_t)0x12345678)

//˽�� macro
//˽�� variables 
uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
uint32_t EEAddrToRead=0x00,EEAddrToWrite=0x00;
uint32_t EEDataReaded=0x00,EEDataToWrite=0x00;

uint8_t EEArray[488];//��Ӧ488��ģ��EE��ַ

void InitFlashEE(void)//��ʼ��FlashEE�����ǲ���Ԥ����4ҳ
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
  NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

  //������0x08006000~0x08007000,��4K BYTE,4ҳ
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
    {
     //��������
			;
    }
  }
	FLASH_Lock();//����flash,��ֹ�Ƿ����� 
}

uint32_t FlashEERead(uint32_t addr)//��flash����
{
	//uint32_t addr1;
	EEAddrToRead = FLASH_USER_START_ADDR + addr;//��0x8000F000��ʼ������Ե�ַ(EEAddrToRead)ָ��ľ��Ե�ַ
	//FLASH_Unlock();//����FLASH,����������Ҫ���� 
//	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);  //�����ȷ����״̬λ
	EEDataReaded = *(volatile uint32_t *)EEAddrToRead;
	//EEAddrToRead = EEAddrToRead - FLASH_USER_START_ADDR;//�ָ���Ե�ַ
	return EEDataReaded;
}

void FlashEEWrite(uint32_t addr,uint32_t data)//дflash����
{
	uint32_t addr1,data1;
	addr1=addr;data1=data;
	EEAddrToWrite =  FLASH_USER_START_ADDR + addr1;//��0x8000F000��ʼ������Ե�ַ(addr)ָ��ľ��Ե�ַ
	FLASH_Unlock();//����FLASH,
	if (FLASH_ProgramWord(EEAddrToWrite,data1) != FLASH_COMPLETE)
	{
		;
	}
	FLASH_Lock();//����flash,��ֹ�Ƿ����� 
	//EEAddrToWrite = EEAddrToWrite - FLASH_USER_START_ADDR;//�ָ���Ե�ַ
	
	//У��д����Ƿ���ȷ
	//data2=FlashEERead(addr1);
	//if(data2==data1){
	//	return 1;
	//}
	//return 0;//д��ʧ��
}
void RefreshEE(void)//ˢ��EE
{
	uint16_t i;
	uint32_t DestEEData;
	//1.����һ��ҳ
	ResetFlashEE(0);
	//2.д��������
	for(i=0;i<488;i+=4){
		DestEEData=(((uint32_t)EEArray[i+3]) << 24) & 0xFF000000;
		DestEEData |= (((uint32_t)EEArray[i+2]) << 16) & 0x00FF0000;
		DestEEData |= (((uint32_t)EEArray[i+1]) << 8) & 0x0000FF00;
		DestEEData |= ((uint32_t)EEArray[i]) & 0x000000FF;
		FlashEEWrite(i,DestEEData);
	}
}
void ResetFlashEE(uint32_t flashpage)//ʹ��FLASH_ErasePage()����flash
{
	FLASH_Unlock();//����FLASH 
	//����FLASH(FLASH_USER_START_ADDR ~ FLASH_USER_END_ADDR)
  //FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);  //�����ȷ����״̬λ
	//FLASH_FLAG_EOP��FLASH�����ϱ�־��FLASH_FLAG_PGERR��FLASH��̴����־��FLASH_FLAG_WRPERR��FLASHд������־
  NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;//Ҫ������ҳ����0x1000 / 0x400 = 4

	if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * flashpage))!= FLASH_COMPLETE)
	{
		//����ʱ����дflash������Ӵ������
		;
		//return FAILED;
	}
	FLASH_Lock();//����flash,��ֹ�Ƿ����� 
  MemoryProgramStatus = PASSED;
	//return PASSED;
}

