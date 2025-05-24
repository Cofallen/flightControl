#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <reg52.h>
#include <intrins.h>

#define uchar unsigned char
#define uint unsigned int

void UART_Init();
void Bluetooth_SendByte(uchar dat);
void Bluetooth_SendString(char *s);

#endif