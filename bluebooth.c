#include "bluebooth.h"

// ���ڳ�ʼ��(����ͨ��)
void UART_Init()
{
    TMOD |= 0x20;    // ��ʱ��1������ʽ2
    TH1 = 0xFD;     // ������9600
    TL1 = 0xFD;
    TR1 = 1;        // ������ʱ��1
    
    SCON = 0x50;    // ���ڹ�����ʽ1���������
    ES = 1;         // �������ж�
    EA = 1;         // �����ж�
}

// ����һ���ֽ�
void Bluetooth_SendByte(uchar dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// �����ַ���
void Bluetooth_SendString(char *s)
{
    while(*s)
    {
        Bluetooth_SendByte(*s++);
    }
}