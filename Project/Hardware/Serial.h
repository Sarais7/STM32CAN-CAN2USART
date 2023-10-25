#ifndef __SERIAL_H__
#define __SERIAL_H__
#include "stm32f10x.h"                  // Device header

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
#endif
