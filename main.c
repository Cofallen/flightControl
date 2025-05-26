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
bit dataReady = 0;         // 数据就绪标志
// 定时器计数
int msTick = 0;
int counter = 0;
// 定时器0初始化函数
void Timer0_Init(void)
{
    TMOD &= 0xF0;    // 清零T0的控制位
    TMOD |= 0x01;    // 设置T0为模式1(16位定时器)
    
    TH0 = (65536-917) / 256;    // 高8位 = 0xFC
    TL0 = (65536-917) % 256;    // 低8位 = 0x18

    PT0 = 0x10;  // 设置串口中断为高优先级
    PS = 0;  
    ET0 = 1;         // 使能定时器0中断
    EA = 1;          // 开总中断
    TR0 = 1;         // 启动定时器0
}

void main()
{
    FlightControl_Init();
    LED2=0;LED3=0;LED1=0;
    // 蓝牙模块需要初始化时间
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
            LED2 = !LED2;  // 切换LED状态

        }
        // Motor_SetSpeed(1, 1000);  // 设置电机1速度为500
        // Motor_SetSpeed(3, 1000);  // 设置电机1速度为500
        // Motor_SetSpeed(2, 1000);  // 设置电机2速度为500
        // Motor_SetSpeed(4, 1000);  // 设置电机2速度为500

        // FlightControl_SetTarget(fc_attitude.target.roll, 
        //                         fc_attitude.target.pitch, 
        //                         fc_attitude.target.yaw, 
        //                         1000);
        // FlightControl_CalculateOutput();
    }
}

// 定时器0中断服务函数
void Timer0_ISR(void) interrupt 1
{
    // 重新加载定时器初值
    TH0 = (65536-917) / 256;    // 0xFC
    TL0 = (65536-917) % 256;    // 0x18

    msTick++;
    Motor_PWM_Update();  

    if(msTick >= 50)  // 每50ms处理一次数据 // 实际为 0.098s
    {
        msTick = 0;
        
        // 获取陀螺仪数据
        FlightControl_UpdateAttitude(&mpu6050_data);
        
        // 转换数据
        roll = (int)(mpu6050_data.roll);
        pitch = (int)(mpu6050_data.pitch);
        yaw = (int)(mpu6050_data.yaw);
        
        // 准备发送数据
        sprintf(buffer, "r: %d, p: %d, y: %d", roll, pitch, yaw);
        dataReady = 1;  // 设置数据就绪标志
        LED1 = !LED1;  // 切换LED状态
    }
}