#include "stm32f10x.h"

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//开启USART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;									//定义GPIO的初始化结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//设置GPIO为推挽输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;							//选择PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//设置GPIO端口速度为50MHz
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	USART_InitTypeDef USART_InitStructure;								//定义USART1的初始化结构体
	USART_InitStructure.USART_BaudRate = 9600;						//设置波特率为9600
																												//不使用流控
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;				//设置为发送模式
	USART_InitStructure.USART_Parity = USART_Parity_No;		//不使用校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//使用1位停止位
																												//字长设置为8bit
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART1,&USART_InitStructure);							//用结构体的参数初始化USART1
	USART_Cmd(USART1, ENABLE);														//启用USART1
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);													//调用USART的数据发送函数
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//调用标志获取函数，监测发送数据寄存器空标志位
																												//当TXE置1，说明数据已经全部传入移位寄存器
}
