# STM32 CAN通信收发的实现

### 1.设置几个全局变量

```
uint16_t CAN_Rid = 0x7e9;
uint8_t RxFlag;
CanRxMsg RxMsg;
```

CAN_Rid：接收信息的ID（只接收该ID的报文）

RxFlag：接收信息的标志位（为1说明接收到信息）

RxMsg：用来存放接收到的信息的结构体

### 2.配置GPIO口

此处使用正点原子[精英]开发板，通过文档查询到该程序使用到的IO端口：

| PA11 | USB_D- | CRX  | Y    | 1，USB  D-引脚(P6设置)     2，CRX引脚(P6设置) | 该IO通过P6选择连接USB  D-还是CAN的RX脚，如果去掉P6的跳线帽，则该IO完全独立 |
| ---- | ------ | ---- | ---- | --------------------------------------------- | ------------------------------------------------------------ |
| PA12 | USB_D+ | CTX  | Y    | 1，USB  D+引脚(P6设置)     2，CTX引脚(P6设置) | 该IO通过P6选择连接USB  D+还是CAN的TX脚，如果去掉P6的跳线帽，则该IO完全独立 |

SourceCode：

```c
void CAN_GPIO_Config(void)		
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;				//定义初始化结构体						
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//CAN_RX配置上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	//CAN_TX配置复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*以上是硬件接口的初始化*/
}
```

此处使用CAN1，而涉及端口为GPIOA的PA11、PA12，故先开启CAN1（在APB1上）和GPIOA（在APB2上）的时钟。

定义GPIO的初始化结构体，并配置其中参数：

PA11为CAN输入端口，配置为上拉输入模式；PA12为CAN输出端口，配置为复用推挽输出。

GPIO_Speed要求不能设置低于信号频率，这里选择最大50MHz

配置完参数后，将结构体地址送入GPIO_Init()中应用到端口。

### 3.配置CAN初始化

SourceCode：

```c
void CAN_INIT(void)							//区别于库函数中的CAN_Init，改成全大写
{	
	CAN_InitTypeDef CAN_InitStructure;		//定义初始化结构体	
	CAN_DeInit(CAN1);						//先复位CAN
	CAN_StructInit(&CAN_InitStructure);		//对结构体赋默认值
	
	CAN_InitStructure.CAN_ABOM = DISABLE;	//不开启自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE; 	//不开启自动唤醒模式
	CAN_InitStructure.CAN_NART = DISABLE; 	//不开启非自动重传模式
	CAN_InitStructure.CAN_RFLM = DISABLE; 	//不开启接受FIFO锁定模式
	CAN_InitStructure.CAN_TTCM = DISABLE; 	//不开启时间触发模式
	CAN_InitStructure.CAN_TXFP = DISABLE; 	//不开启发送FIFO优先级
		
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN正常模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//定义位时序各段长度，可由此确定波特率
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 60;
	/*波特率 = 72MHz/2/60*(CAN_BS1_3tq + CAN_BS2_2tq + CAN_SJW_1tq) 
			= 72MHz/2/60*(3+2+1) 
			= 100kHz*/
	CAN_Init(CAN1, &CAN_InitStructure);	
    /*以上为CAN1的初始化
}
```

此处使用CAN1，先对CAN1进行复位。

定义CAN的初始化结构体，并配置其中参数：

对初始化结构体赋默认值。

CAN1的特殊功能都设置为DISABLE（不启用）。

CAN的工作模式（CAN_Mode）设置为普通模式，即可正常收/发信息。

CAN的位时序各位分别设置参数。通过这四个值和STM32的主频（72MH）可得出波特率的数值。

配置完参数后，将结构体地址送入CAN_Init()中应用到CAN。

### 4.配置过滤器

SourceCode：

```c
void CAN_Filter_INIT(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;//定义过滤器初始化结构体
    CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_Rid << 5;
    //只接收前11 bit与std_id（0x7e9）完全相同的消息
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000 | CAN_ID_STD;

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;
	//接收器会检查mask为1的位中，FilterId和输入信息的Id是否一致
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	//由于标准数据帧中，std_id信息被储存在高16位（的高位11 bit中），所以低16位不用考虑

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //指向过滤器的是FIFO0，即使用CAN_FIFO0来缓存接收数据
	CAN_FilterInitStructure.CAN_FilterNumber = 1;  //指定使用过滤器组1，范围是0~13
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //指定过滤器为标识符屏蔽位模式（常用）
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //过滤器位宽32 bit(标准数据帧，扩展数据帧都能支持)
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //使能过滤器

	CAN_FilterInit(&CAN_FilterInitStructure);
    
}
```

先定义过滤器初始化结构体，并配置其中参数：

过滤器高位的前11位（在16位中前移5位刚好在最前）设置为Rid（接收报文ID），其余位置0，表示报文==前11位（即报文ID）==要与Rid相同。过滤器低位不需要使用，设置为默认值（STD_ID）。

过滤器掩码高位设置为0xFFE0（二进制形式为1111111111100000，即将前11位为1），表示接收器只检查输入信息的==前11位==与过滤器中是否相同。低位不需要使用，设置为0。

为过滤器分配的FIFO选择==FIFO0==，使用的过滤器组选择==过滤器组1==。

过滤器工作模式选择标识符==屏蔽位==模式，过滤器位宽选择==32bit==（标准或扩展数据帧都支持）

过滤器使能设置为==ENABLE==（即使能过滤器）

配置完参数后，将结构体地址送入CAN_FilterInit()中应用到过滤器。

### 5.配置CAN的中断

SourceCode:

```C
void CAN_NVIC_Config(void)
{
	/*配置NVIC中断优先级，初始化NVIC*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; 
    //本型号STM32的CAN1_RX0中断源和USB_LP共用一个中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //优先级随便设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
    //USB_LP_CAN1_RX0_IRQn标志位一旦置1，就会向NVIC申请中断
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
```

配置优先级分组为组2（先占优先级2位，从优先级2位）

定义NVIC初始化结构体，并配置其中参数：

IRQ通道（即选择哪个中断）选择USB_LP_CAN1_RX0_IRQn（在F10x型号中，CAN1_RX0中断源和USB_LP共用一个中断通道）。

占优先级和从优先级设置为0

中断使能设置为ENABLE

配置完参数后，将结构体地址送入NVIC_Init()中应用到中断。

然后使能CAN中断FMP0（FMP0：FIFO0 消息挂号中断屏蔽）

### <font color="red">至此，CAN初始化结束</font>

### 6.配置发送函数

SourceCode：

```c
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
```

定义发送报文初始化结构体，并配置其参数：。

发送报文数据配置为data（==此处只发送1字节作为示例，最多可发送8字节==）。

DLC位配置为1，即数据位长度1字节。

ExtID为0，IDE段配置为标准ID（CAN_Id_Standard），即使用标准数据帧而非扩展数据帧。

RTR段配置为数据帧（CAN_RTR_Data），即使用数据帧。

StdId配置为向函数中送入的发送ID（CAN_Tid）。

调用CAN传输函数（CAN_Transmit()），将结构体地址送入，将信息发送到邮箱中。



### 7.配置中断接收函数

对于STM32标准库，中断函数在触发对应的中断时，是会自动调用的。

在启动文件startup_stm32f10x_md.s中查找到

```C
DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
```

即在该型号STM32中，CAN1的接收和USB低优先级使用的是同一个中断

故中断接收函数SourceCode：

```C
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

```

当触发中断时，调用CAN中断检测函数（CAN_GetITStatus()），检测CAN1的FMP0中断是否触发。

若FMP0触发，表示接收到了数据。将FIFO0中接收到的数据存入==接收信息结构体==（RxMsg）中，将接受标志位置1，最后清除FMP0的标志位。

### 8.接受信息处理

```C
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
```

在中断函数中接受完信息后，需要将接收到的信息传出。

在其他函数中，调用CAN_GetRxFlag()即读取接受标志位，读取完清零。

调用CAN_GetRxData()即读取接收信息结构体。

### 9.CAN的使用

按照顺序分别配置GPIO口、CAN初始化、过滤器、中断。

需要发送信息时，将数据和发送ID传入CAN_SendByte()。

需要接收信息时，需要定义一个用来存放接收信息的结构体，检测接收信息标志位。若检测到标志位被置1，调用CAN_GetRxData()，并将获取到的数据（返回值）存入定义的结构体中。

