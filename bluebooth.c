#include "bluebooth.h"

// 串口初始化(蓝牙通信)
void UART_Init()
{
    TMOD |= 0x20;    // 定时器1工作方式2
    TH1 = 0xFD;     // 波特率9600
    TL1 = 0xFD;
    TR1 = 1;        // 启动定时器1
    
    SCON = 0x50;    // 串口工作方式1，允许接收
    ES = 1;         // 允许串口中断
    EA = 1;         // 开总中断
}

// 发送一个字节
void Bluetooth_SendByte(uchar dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// 发送字符串
void Bluetooth_SendString(char *s)
{
    while(*s)
    {
        Bluetooth_SendByte(*s++);
    }
}