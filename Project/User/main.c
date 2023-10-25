#include "Serial.h"
#include "CAN.h"
#include "Delay.h"
int main(void)
{
	Serial_Init();
	CAN_GPIO_Config();
	CAN_RT_Init();
	CAN_NVIC_Config();
	
	CanRxMsg RxMsg;
	
//	CAN_SendByte(0xFF,0x777);
	while(1)
	{
		if(CAN_GetRxFlag() == 1)
		{
			RxMsg = CAN_GetRxData();
			Delay_ms(1000);
			//CAN_SendByte(RxMsg.Data[0],0x777);
			Serial_SendByte(RxMsg.Data[0]);
		}
	}
//	CanRxMsg RxMsg;
//	while(1)
//	{
//		if(USB_LP_CAN1_RX0_IRQHandler(&RxMsg) == 1)
//		{
//			Serial_SendByte(RxMsg.Data[0]);
//		}
//	}
	
}
