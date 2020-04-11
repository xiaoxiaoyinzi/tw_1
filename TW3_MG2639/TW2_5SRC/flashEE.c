#include "main.h"

// 定义私有枚举类型
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
// 私有的define 
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   //每页为1K BYTE
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800F000)   //芯片Flash的起始地址为 0x08000000
#define FLASH_USER_END_ADDR     ((uint32_t)0x0800FFFF)   //用作data存储的地址区间：0x0800F000~0x0800FFFF,共4K BYTE
#define DATA_32                 ((uint32_t)0x12345678)

//私有 macro
//私有 variables 
uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
uint32_t EEAddrToRead=0x00,EEAddrToWrite=0x00;
uint32_t EEDataReaded=0x00,EEDataToWrite=0x00;

uint8_t EEArray[488];//对应488个模拟EE地址

void InitFlashEE(void)//初始化FlashEE，就是擦除预定的4页
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
  NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

  //擦除从0x08006000~0x08007000,共4K BYTE,4页
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
    {
     //擦除错误
			;
    }
  }
	FLASH_Lock();//锁定flash,防止非法访问 
}

uint32_t FlashEERead(uint32_t addr)//读flash操作
{
	//uint32_t addr1;
	EEAddrToRead = FLASH_USER_START_ADDR + addr;//从0x8000F000开始计算相对地址(EEAddrToRead)指向的绝对地址
	//FLASH_Unlock();//解锁FLASH,读操作不需要解锁 
//	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);  //清除不确定的状态位
	EEDataReaded = *(volatile uint32_t *)EEAddrToRead;
	//EEAddrToRead = EEAddrToRead - FLASH_USER_START_ADDR;//恢复相对地址
	return EEDataReaded;
}

void FlashEEWrite(uint32_t addr,uint32_t data)//写flash操作
{
	uint32_t addr1,data1;
	addr1=addr;data1=data;
	EEAddrToWrite =  FLASH_USER_START_ADDR + addr1;//从0x8000F000开始计算相对地址(addr)指向的绝对地址
	FLASH_Unlock();//解锁FLASH,
	if (FLASH_ProgramWord(EEAddrToWrite,data1) != FLASH_COMPLETE)
	{
		;
	}
	FLASH_Lock();//锁定flash,防止非法访问 
	//EEAddrToWrite = EEAddrToWrite - FLASH_USER_START_ADDR;//恢复相对地址
	
	//校验写入的是否正确
	//data2=FlashEERead(addr1);
	//if(data2==data1){
	//	return 1;
	//}
	//return 0;//写入失败
}
void RefreshEE(void)//刷新EE
{
	uint16_t i;
	uint32_t DestEEData;
	//1.擦除一整页
	ResetFlashEE(0);
	//2.写入新数据
	for(i=0;i<488;i+=4){
		DestEEData=(((uint32_t)EEArray[i+3]) << 24) & 0xFF000000;
		DestEEData |= (((uint32_t)EEArray[i+2]) << 16) & 0x00FF0000;
		DestEEData |= (((uint32_t)EEArray[i+1]) << 8) & 0x0000FF00;
		DestEEData |= ((uint32_t)EEArray[i]) & 0x000000FF;
		FlashEEWrite(i,DestEEData);
	}
}
void ResetFlashEE(uint32_t flashpage)//使用FLASH_ErasePage()擦除flash
{
	FLASH_Unlock();//解锁FLASH 
	//擦除FLASH(FLASH_USER_START_ADDR ~ FLASH_USER_END_ADDR)
  //FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);  //清除不确定的状态位
	//FLASH_FLAG_EOP：FLASH编程完毕标志，FLASH_FLAG_PGERR：FLASH编程错误标志，FLASH_FLAG_WRPERR：FLASH写保护标志
  NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;//要擦除的页数：0x1000 / 0x400 = 4

	if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * flashpage))!= FLASH_COMPLETE)
	{
		//擦除时发生写flash错误，添加处理代码
		;
		//return FAILED;
	}
	FLASH_Lock();//锁定flash,防止非法访问 
  MemoryProgramStatus = PASSED;
	//return PASSED;
}

