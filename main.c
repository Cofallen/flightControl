#include "mpu6050.h"
#include <reg52.h>
#include <stdio.h>
#include <bluebooth.h>
#include "motor.h"
#include "fc.h"
#include "main.h"
#include "STRING.H"

#define BUFFER_SIZE 35

sbit LED1=P0^1;
sbit LED2=P0^2;
sbit LED3=P0^3;
sbit M1=P1^0;
sbit M2=P1^1;
sbit M3=P1^2;
sbit M4=P1^3;

int roll, pitch, yaw;

MPU6050_Data_t mpu6050_data;
char buffer[BUFFER_SIZE];  // 将buffer定义为全局变量，以便在中断中使用
bit dataReady = 0;         // 数据就绪标志
// 定时器计数，每100ms处理一次数据
int msTick = 0;
int   counter = 0;
// 定时器0初始化函数，1ms中断一次
void Timer0_Init(void)
{
    TMOD &= 0xF0;    // 清零T0的控制位
    TMOD |= 0x01;    // 设置T0为模式1(16位定时器)
    
    // 设置初值，对于12MHz晶振，1ms定时值为65536-1000
    TH0 = (65536-1000) / 256;    // 高8位 = 0xFC
    TL0 = (65536-1000) % 256;    // 低8位 = 0x18

    PT0 = 0x10;  // 设置串口中断为高优先级
    PS = 0;  
    ET0 = 1;         // 使能定时器0中断
    EA = 1;          // 开总中断
    TR0 = 1;         // 启动定时器0
}

void main()
{
    I2C_Init();
    MPU6050_Init();

    M1=0;M2=0;M3=0;M4=0;LED2=0;LED3=0;LED1=0;
    // 蓝牙模块可能需要初始化时间
    Delay_ms(1000);
    MPU6050_CalibrateInit();
    Delay_ms(100);

    UART_Init();     // 初始化串口(蓝牙)
    Timer0_Init();   // 初始化定时器0
    Bluetooth_SendString("MPU6050 Data Ready (with Complementary Filter)\r\n");
    
    while(1)
    {
        // 当数据就绪时发送
        if(dataReady)
        {
            Bluetooth_SendString(buffer);
            dataReady = 0;  // 清除就绪标志
        }

        // 添加定时主循环测试发送
        Bluetooth_SendString("Main loop test");
        Delay_ms(1000);  // 每秒发送一次
        LED2 = !LED2;  // 切换LED状态
    }
}

// 定时器0中断服务函数
void Timer0_ISR(void) interrupt 1
{
    // 重新加载定时器初值
    TH0 = (65536-1000) / 256;    // 0xFC
    TL0 = (65536-1000) % 256;    // 0x18

    msTick++;
    
    if(msTick >= 100)  // 每100ms处理一次数据
    {
        msTick = 0;
        
        // 获取陀螺仪数据
        FlightControl_UpdateAttitude(&mpu6050_data);
        
        // 转换数据
        roll = (int)(mpu6050_data.roll);
        pitch = (int)(mpu6050_data.pitch);
        yaw = (int)(mpu6050_data.yaw);
        
        // 准备发送数据
        sprintf(buffer, "roll: %d, pitch: %d, yaw: %d\r\n", roll, pitch, yaw);
        dataReady = 1;  // 设置数据就绪标志
        LED1 = !LED1;  // 切换LED状态
    }
}