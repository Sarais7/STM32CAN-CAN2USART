#include "stm32f10x.h"
#include "Serial.h"

uint16_t CAN_Rid = 0x7e9;
uint8_t RxFlag;
CanRxMsg RxMsg;


void CAN_GPIO_Config(void)															//CAN的硬件接口定义
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//开启CAN1和GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;									//定义GPIO的初始化结构体
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					//CAN_RX配置为上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 			//CAN_TX配置为复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*以上是硬件接口的初始化*/
}

void CAN_RT_Init(void)
{	
	CAN_InitTypeDef CAN_InitStructure;										//定义CAN以及过滤器的初始化结构体
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	CAN_DeInit(CAN1);																			//先复位CAN
	CAN_StructInit(&CAN_InitStructure);										//对结构体赋默认值
	
	CAN_InitStructure.CAN_ABOM = DISABLE;									//不开启自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE; 								//不开启自动唤醒模式
	CAN_InitStructure.CAN_NART = DISABLE; 								//不开启非自动重传模式
	CAN_InitStructure.CAN_RFLM = DISABLE; 								//不开启接受FIFO锁定模式
	CAN_InitStructure.CAN_TTCM = DISABLE; 								//不开启时间触发模式
	CAN_InitStructure.CAN_TXFP = DISABLE; 								//不开启发送FIFO优先级
	
	
	
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 				//CAN正常模式
	CAN_InitStructure.CAN_SJW =  CAN_SJW_1tq;							//定义位时序各段长度，可由此确定波特率
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 60;
	/*波特率 = 72MHz/2/60*(CAN_BS1_3tq + CAN_BS2_2tq + CAN_SJW_1tq) = 72MHz/2/60*(3+2+1) = 100kHz*/
	CAN_Init(CAN1, &CAN_InitStructure);											//使用结构体参数初始化CAN1
	
	
	
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_Rid << 5; //只接收前11 bit与std_id（0x7e9）完全相同的消息
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000 | CAN_ID_STD;

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;
	//MASK全为0则表示接受一切消息，不做屏蔽，接收器会检查mask为1的位中，FilterId和输入信息的Id是否一致
	//CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	//由于标准数据帧中，std_id信息被储存在高16位（的高位11 bit中），所以低16位不用考虑（低3 bit懒得检测了）

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //指向过滤器的是FIFO0，即使用CAN_FIFO0来缓存接收数据
	CAN_FilterInitStructure.CAN_FilterNumber = 1;  //指定使用过滤器组1，范围是0~13
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //指定过滤器为标识符屏蔽位模式（常用）
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //过滤器位宽32 bit(标准数据帧，扩展数据帧都能支持)
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //使能过滤器

	CAN_FilterInit(&CAN_FilterInitStructure);
	
}

void CAN_NVIC_Config(void)
{
	/*配置NVIC中断优先级，初始化NVIC*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //本型号STM32的CAN1_RX0中断源和USB_LP共用一个中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //优先级随便设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); //USB_LP_CAN1_RX0_IRQn标志位一旦置1，就会向NVIC申请中断
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}

//发送函数
//uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
void CAN_SendByte(uint8_t data, uint32_t CAN_Tid)
{
	/*发送需要配置CanTxMsg报文结构体*/
	CanTxMsg TxMsg;

	TxMsg.Data[0] = data;
	TxMsg.DLC = 1;
	TxMsg.ExtId = 0x0000;
	TxMsg.IDE = CAN_Id_Standard;
	TxMsg.RTR = CAN_RTR_Data; //数据帧，表示发送数据，远程帧CAN_RTR_Remote表示接受
	TxMsg.StdId = CAN_Tid; //uint32_t StdId 发送数据的标识符

	uint8_t mbox =  CAN_Transmit(CAN1, &TxMsg);
}


//中断接受，对应的中断函数名在启动文件startup_stm32f10x_md.s中查询
//void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	/*检测中断标志位*/
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
	{
		
		CAN_Receive(CAN1, CAN_FIFO0, &RxMsg); //使用CAN_FIFO0来缓存接收数据
		RxFlag = 1;
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0); //最后别忘了清除标志位
	}
}

uint8_t CAN_GetRxFlag(void)
{
	if (RxFlag == 1)
	{
		RxFlag = 0;
		return 1;
	}
	return 0;
}

CanRxMsg CAN_GetRxData(void)
{
	return RxMsg;
}

