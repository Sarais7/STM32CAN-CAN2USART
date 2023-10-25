#ifndef __CAN_H__
#define __CAN_H__
#include "stm32f10x.h"                  // Device header


void CAN_RT_Init(void);
void CAN_GPIO_Config(void);
void CAN_NVIC_Config(void);
void CAN_SendByte(uint8_t data, uint32_t CAN_Tid);

uint8_t CAN_GetRxFlag(void);
CanRxMsg CAN_GetRxData(void);
#endif
