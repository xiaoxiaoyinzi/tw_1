//����˽��define
#define BRT_DISABLE()         GPIO_SetBits(GPIOC, GPIO_Pin_13)//����BRT,֪ͨBLEģ�����ݷ������(��������)
#define BRT_ENABLE()          GPIO_ResetBits(GPIOC, GPIO_Pin_13)//����BRT,֪ͨBLEģ�������ݷ���
#define BRE_ENABLED()         (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14) == 0)//ģ����Խ�������
#define BRE_DISABLED()        (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14) == 1)//ģ�鲻���Խ�������
#define BTT_ENABLED()         (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) == 0)//ģ�鷢������
#define BTT_DISABLED()        (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) == 1)//ģ�������ݷ���
#define BT_CTL_DISABLE()      GPIO_SetBits(GPIOF, GPIO_Pin_0)//֪ͨBLEģ��Ͽ���������
#define BT_CTL_ENABLE()       GPIO_ResetBits(GPIOF, GPIO_Pin_0)//֪ͨBLEģ�鿪ʼ��������
#define MODE_CTL_DATA()       GPIO_SetBits(GPIOF, GPIO_Pin_1)//֪ͨBLEģ���л���͸��ģʽ
#define MODE_CTL_CMD()       GPIO_ResetBits(GPIOF, GPIO_Pin_1)//֪ͨBLEģ���л���ָ��ģʽ
#define CONN_STAT_OFF()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == 0)//ģ��������δ����
#define CONN_STAT_ON()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == 1)//ģ��������������
#define REST_N_RUN()          GPIO_SetBits(GPIOA, GPIO_Pin_10)//����REST_N��ģ����������
#define REST_N_RESET()        GPIO_ResetBits(GPIOA, GPIO_Pin_10)//����REST_N�Ը�λBLEģ��
#define STAT_INDY_OFF()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 0)//ģ��״̬֪ͨ
#define STAT_INDY_ON()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 1)//ģ��״̬֪ͨ���ϵ������2S
//#define STAT_NOTBUSY()       (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) == 0)//ģ�鲻æ
//#define STAT_BUSY()        (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) == 1)//ģ��æ

//����˽�б���
unsigned char DoNothing; //һ��ʲôҲ�����ı�����ʵ��NOPռλ����
unsigned char Clock1s; //1s��ʱ��
unsigned char Clock100ms; //10ms��ʱ��

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
volatile unsigned char  ProUsart1; //Э����������־��0x55���������0xAA���������
unsigned char Usart1ErrFlag; //����1ͨѶ���ϱ�־��0x55����ͨѶ����,0xAAͨѶ����
volatile unsigned char Usart1OrderMode; //�������ͣ�
volatile unsigned char Usart1OrderWait; //��ʱ����������ģ�鷢��һ�������ó�3������޻ظ���ÿ��SysTick��1
volatile unsigned char Usart1OrderTimeOut; //Usart1OrderWait����0ʱ��Usart1OrderTimeOut+1
unsigned char Usart1OrderErr; //Usart1OrderTimeOut�ӵ�3ʱ,�ó�0x55��ʾһ��ͨѶ����,����Ϊ0xAA 

volatile unsigned char  Tx2Buf[40];//USART1�������� 
volatile unsigned char  Rx2Buf[40];//USART1��������
volatile unsigned char  Tx2Read;
volatile unsigned char  CntTx2;//USART1���ͼ�����
volatile unsigned char  CntRx2;//USART1���յ����ֽ��� 
volatile unsigned char  NbrOfDataToTrans2;//��Ҫ���͵��ֽ���
volatile unsigned char  NbrOfDataToRead2;
volatile unsigned char  Rx2Start; //����״̬��־��0x55��ʼ���գ�0xAA�������
volatile unsigned char  Rx2Read; //����Э��ʱ��ȡ���ݵ�����
volatile unsigned char  U1DrCopy2; //USART1������DR�ı���
volatile unsigned char  ProUsart2; //Э����������־��0x55���������0xAA���������
unsigned char Usart2ErrFlag; //����1ͨѶ���ϱ�־��0x55����ͨѶ����,0xAAͨѶ����
volatile unsigned char Usart2OrderMode; //�������ͣ�
volatile unsigned char Usart2OrderWait; //��ʱ����������ģ�鷢��һ�������ó�3������޻ظ���ÿ��SysTick��1
volatile unsigned char Usart2OrderTimeOut; //Usart1OrderWait����0ʱ��Usart1OrderTimeOut+1
unsigned char Usart2OrderErr; //Usart1OrderTimeOut�ӵ�3ʱ,�ó�0x55��ʾһ��ͨѶ����,����Ϊ0xAA 

unsigned char  UsartErrData; //USART1��USART2�յ��Ĵ�������



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