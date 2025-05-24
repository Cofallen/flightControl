#include "mpu6050.h"
#include <reg52.h>
#include <stdio.h>
#include <bluebooth.h>
#include "motor.h"
#include "fc.h"
#include "main.h"
#include "STRING.H"

#define BUFFER_SIZE 15

sbit LED1=P0^1;
sbit LED2=P0^2;
sbit LED3=P0^3;

int roll, pitch, yaw;

char buffer[BUFFER_SIZE];  // 
bit dataReady = 0;         // ���ݾ�����־
// ��ʱ������
int msTick = 0;
int counter = 0;
// ��ʱ��0��ʼ������
void Timer0_Init(void)
{
    TMOD &= 0xF0;    // ����T0�Ŀ���λ
    TMOD |= 0x01;    // ����T0Ϊģʽ1(16λ��ʱ��)
    
    // ���ó�ֵ������12MHz����1ms��ʱֵΪ65536-1000
    TH0 = (65536-1000) / 256;    // ��8λ = 0xFC
    TL0 = (65536-1000) % 256;    // ��8λ = 0x18

    PT0 = 0x10;  // ���ô����ж�Ϊ�����ȼ�
    PS = 0;  
    ET0 = 1;         // ʹ�ܶ�ʱ��0�ж�
    EA = 1;          // �����ж�
    TR0 = 1;         // ������ʱ��0
}

void main()
{
    FlightControl_Init();
    LED2=0;LED3=0;LED1=0;
    // ����ģ����Ҫ��ʼ��ʱ��
    Delay_ms(1000);
    MPU6050_CalibrateInit();
    Delay_ms(100);

    UART_Init();     // ��ʼ������(����)
    Timer0_Init();   // ��ʼ����ʱ��0
    Bluetooth_SendString("MPU6050 Data Ready (with Complementary Filter)\r\n");
    
    while(1)
    {
        // �����ݾ���ʱ����
        if(dataReady)
        {
            Bluetooth_SendString(buffer);
            dataReady = 0;  // ���������־
            LED2 = !LED2;  // �л�LED״̬

        }
    }
}

// ��ʱ��0�жϷ�����
void Timer0_ISR(void) interrupt 1
{
    // ���¼��ض�ʱ����ֵ
    TH0 = (65536-917) / 256;    // 0xFC
    TL0 = (65536-917) % 256;    // 0x18

    msTick++;
    
    if(msTick >= 50)  // ÿ50ms����һ������ // ʵ��Ϊ 0.098s
    {
        msTick = 0;
        
        // ��ȡ����������
        FlightControl_UpdateAttitude(&mpu6050_data);
        
        // ת������
        roll = (int)(mpu6050_data.roll);
        pitch = (int)(mpu6050_data.pitch);
        yaw = (int)(mpu6050_data.yaw);
        
        // ׼����������
        sprintf(buffer, "r: %d, p: %d, y: %d", roll, pitch, yaw);
        dataReady = 1;  // �������ݾ�����־
        LED1 = !LED1;  // �л�LED״̬
    }
}