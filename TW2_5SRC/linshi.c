//定义私有define
#define BRT_DISABLE()         GPIO_SetBits(GPIOC, GPIO_Pin_13)//拉高BRT,通知BLE模块数据发送完成(或无数据)
#define BRT_ENABLE()          GPIO_ResetBits(GPIOC, GPIO_Pin_13)//拉低BRT,通知BLE模块有数据发送
#define BRE_ENABLED()         (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14) == 0)//模块可以接收数据
#define BRE_DISABLED()        (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14) == 1)//模块不可以接收数据
#define BTT_ENABLED()         (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) == 0)//模块发送数据
#define BTT_DISABLED()        (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) == 1)//模块无数据发送
#define BT_CTL_DISABLE()      GPIO_SetBits(GPIOF, GPIO_Pin_0)//通知BLE模块断开蓝牙连接
#define BT_CTL_ENABLE()       GPIO_ResetBits(GPIOF, GPIO_Pin_0)//通知BLE模块开始蓝牙连接
#define MODE_CTL_DATA()       GPIO_SetBits(GPIOF, GPIO_Pin_1)//通知BLE模块切换到透传模式
#define MODE_CTL_CMD()       GPIO_ResetBits(GPIOF, GPIO_Pin_1)//通知BLE模块切换到指令模式
#define CONN_STAT_OFF()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == 0)//模块与主机未连接
#define CONN_STAT_ON()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == 1)//模块与主机已连接
#define REST_N_RUN()          GPIO_SetBits(GPIOA, GPIO_Pin_10)//拉高REST_N，模块正常运行
#define REST_N_RESET()        GPIO_ResetBits(GPIOA, GPIO_Pin_10)//拉低REST_N以复位BLE模块
#define STAT_INDY_OFF()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 0)//模块状态通知
#define STAT_INDY_ON()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 1)//模块状态通知，上电后拉高2S
//#define STAT_NOTBUSY()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) == 0)//模块不忙
//#define STAT_BUSY()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) == 1)//模块忙

//定义私有变量
unsigned char DoNothing; //一个什么也不做的变量，实现NOP占位功能
unsigned char Clock1s; //1s定时器
unsigned char Clock100ms; //10ms定时器

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
volatile unsigned char  ProUsart1; //协议解析允许标志，0x55允许解析，0xAA不允许解析
unsigned char Usart1ErrFlag; //串口1通讯故障标志：0x55发生通讯故障,0xAA通讯正常
volatile unsigned char Usart1OrderMode; //命令类型：
volatile unsigned char Usart1OrderWait; //超时计数器，向模块发送一次命令置成3，如果无回复，每个SysTick减1
volatile unsigned char Usart1OrderTimeOut; //Usart1OrderWait减到0时，Usart1OrderTimeOut+1
unsigned char Usart1OrderErr; //Usart1OrderTimeOut加到3时,置成0x55表示一次通讯故障,正常为0xAA 

volatile unsigned char  Tx2Buf[40];//USART1发送数组 
volatile unsigned char  Rx2Buf[40];//USART1接收数组
volatile unsigned char  Tx2Read;
volatile unsigned char  CntTx2;//USART1发送计数器
volatile unsigned char  CntRx2;//USART1接收到的字节数 
volatile unsigned char  NbrOfDataToTrans2;//将要发送的字节数
volatile unsigned char  NbrOfDataToRead2;
volatile unsigned char  Rx2Start; //接收状态标志，0x55开始接收，0xAA接收完成
volatile unsigned char  Rx2Read; //解析协议时读取数据的索引
volatile unsigned char  U1DrCopy2; //USART1接收器DR的备份
volatile unsigned char  ProUsart2; //协议解析允许标志，0x55允许解析，0xAA不允许解析
unsigned char Usart2ErrFlag; //串口1通讯故障标志：0x55发生通讯故障,0xAA通讯正常
volatile unsigned char Usart2OrderMode; //命令类型：
volatile unsigned char Usart2OrderWait; //超时计数器，向模块发送一次命令置成3，如果无回复，每个SysTick减1
volatile unsigned char Usart2OrderTimeOut; //Usart1OrderWait减到0时，Usart1OrderTimeOut+1
unsigned char Usart2OrderErr; //Usart1OrderTimeOut加到3时,置成0x55表示一次通讯故障,正常为0xAA 

unsigned char  UsartErrData; //USART1或USART2收到的错误数据



//uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
//void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
//FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint32_t USART_FLAG);
/**
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: where x can be 1 or 2 to select the USART peripheral.
  * @param  USART_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg USART_FLAG_REACK:  Receive Enable acknowledge flag.
  *            @arg USART_FLAG_TEACK:  Transmit Enable acknowledge flag.
  *            @arg USART_FLAG_WU:  Wake up flag.
  *            @arg USART_FLAG_RWU:  Receive Wake up flag.
  *            @arg USART_FLAG_SBK:  Send Break flag.
  *            @arg USART_FLAG_CM:  Character match flag.
  *            @arg USART_FLAG_BUSY:  Busy flag.
  *            @arg USART_FLAG_ABRF:  Auto baud rate flag.
  *            @arg USART_FLAG_ABRE:  Auto baud rate error flag.
  *            @arg USART_FLAG_EOB:  End of block flag.
  *            @arg USART_FLAG_RTO:  Receive time out flag.
  *            @arg USART_FLAG_nCTSS:  Inverted nCTS input bit status.
  *            @arg USART_FLAG_CTS:  CTS Change flag.
  *            @arg USART_FLAG_LBD:  LIN Break detection flag.
  *            @arg USART_FLAG_TXE:  Transmit data register empty flag.
  *            @arg USART_FLAG_TC:  Transmission Complete flag.
  *            @arg USART_FLAG_RXNE:  Receive data register not empty flag.
  *            @arg USART_FLAG_IDLE:  Idle Line detection flag.
  *            @arg USART_FLAG_ORE:  OverRun Error flag.
  *            @arg USART_FLAG_NE:  Noise Error flag.
  *            @arg USART_FLAG_FE:  Framing Error flag.
  *            @arg USART_FLAG_PE:  Parity Error flag.
  * @retval The new state of USART_FLAG (SET or RESET).
  */
//void USART_ClearFlag(USART_TypeDef* USARTx, uint32_t USART_FLAG);