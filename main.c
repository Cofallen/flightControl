#include "mpu6050.h"
#include <reg52.h>
#include <stdio.h>
#include <bluebooth.h>
#include "motor.h"
#include "fc.h"
#include "main.h"
#include "STRING.H"

#define BUFFER_SIZE 16

sbit LED1=P0^1;
sbit LED2=P0^2;
sbit LED3=P0^3;

int roll, pitch, yaw;
void vofa_send(float a1, float a2, float a3);

union buffer_t
{
    unsigned char DATA[BUFFER_SIZE];  // 数据区
    float a[3]; // 浮点数数组
} buffer= {0};

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
            Bluetooth_SendString(buffer.DATA);
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
        FlightControl_CalculateOutput();
        vofa_send(fc_attitude.target.roll, fc_attitude.target.pitch, fc_attitude.target.yaw);
        Delay_ms(10);  // 延时10ms
    }
}

// 定时器0中断服务函数
void Timer0_ISR(void) interrupt 1
{
    // 重新加载定时器初值
    TH0 = (65536-917) / 256;    // 0xFC
    TL0 = (65536-917) % 256;    // 0x18

    msTick++;
    // Motor_PWM_Update();  

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
        sprintf(buffer.DATA, "r: %d, p: %d, y: %d", roll, pitch, yaw);
        dataReady = 1;  // 设置数据就绪标志
        LED1 = !LED1;  // 切换LED状态
    }
}


// ...existing code...

// 声明接收数据变量
char rxData = 'S';  // 默认值为'S'，表示停止

// 串口中断服务函数
void UART_ISR(void) interrupt 4
{
    // 判断是否为接收中断
    if(RI)
    {
        RI = 0;              // 清除接收中断标志
        rxData = SBUF;       // 读取接收到的数据
        
        ProcessCommand(rxData, &fc_attitude, &fc_control);
        // 收到数据后LED3闪烁
        LED3 = !LED3;        // 切换LED3状态
    }
}


void vofa_send(float a1, float a2, float a3)
{
    buffer.a[0] = a1;
    buffer.a[1] = a2;
    buffer.a[2] = a3;
    buffer.DATA[BUFFER_SIZE - 4] = 0X00; 
    buffer.DATA[BUFFER_SIZE - 3] = 0X00;
    buffer.DATA[BUFFER_SIZE - 2] = 0X80;
    buffer.DATA[BUFFER_SIZE - 1] = 0X7f;

    Bluetooth_SendString(buffer.DATA);
}