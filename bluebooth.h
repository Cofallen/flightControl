#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <reg52.h>
#include <intrins.h>
#include "fc.h"

#define uchar unsigned char
#define uint unsigned int

void UART_Init();
void Bluetooth_SendByte(uchar dat);
void Bluetooth_SendString(char *s);
void ProcessCommand(char cmd, FC_Attitude_t *attitude, FC_Control_t *control);

#endif