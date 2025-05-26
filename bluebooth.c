#include "bluebooth.h"
#include "fc.h"

// 串口初始化(蓝牙通信)
void UART_Init()
{
    TMOD &= 0x0F;
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


// static int throttle = 0;    
#define STEP_THROTTLE 50     // 油门调整步长
#define STEP_ATTITUDE 5      // 姿态调整步长
// 串口命令处理函数 - 使用形参传递fc_attitude
void ProcessCommand(char cmd, FC_Attitude_t *attitude, FC_Control_t *control)
{
    switch(cmd)
    {
        case 'G':  // 油门加大
            if(control->throttle  <= 950) control->throttle  += STEP_THROTTLE;
            if(control->throttle  > 1000) control->throttle  = 1000;
            break;
            
        case 'K':  // 油门减小
            if(control->throttle  >= 50) control->throttle  -= STEP_THROTTLE;
            else control->throttle  = 0;
            break;
            
        case 'H':  // 左旋(增加偏航角)
            attitude->target.yaw += STEP_ATTITUDE;
            if(attitude->target.yaw > 180.0f) attitude->target.yaw -= 360.0f;
            break;
            
        case 'J':  // 右旋(减小偏航角)
            attitude->target.yaw -= STEP_ATTITUDE;
            if(attitude->target.yaw < -180.0f) attitude->target.yaw += 360.0f;
            break;
            
        case 'B':  // 前进(减小俯仰角)
            attitude->target.pitch = -STEP_ATTITUDE;
            break;
            
        case 'E':  // 后退(增加俯仰角)
            attitude->target.pitch = STEP_ATTITUDE;
            break;
            
        case 'A':  // 向左倾斜(减小横滚角)
            attitude->target.roll = -STEP_ATTITUDE;
            break;
            
        case 'D':  // 向右倾斜(增加横滚角)
            attitude->target.roll = STEP_ATTITUDE;
            break;
            
        case 'S':  // 停止/恢复水平
            attitude->target.roll = 0.0f;
            attitude->target.pitch = 0.0f;
            break;
            
        case 'I':  // 紧急停止
        control->throttle  = 0;
            attitude->target.roll = 0.0f;
            attitude->target.pitch = 0.0f;
            attitude->target.yaw = 0.0f;
            FlightControl_EmergencyStop();
            break;
            
        default:
            break;
    }
    
    // 更新飞控目标油门
    // control->throttle = throttle;
    
    // 如果油门大于最小值且当前未解锁，则解锁飞控
    if(control->throttle  > 100 && !control->armed)
    {
        FlightControl_Arm(1);  // 解锁
    }
    // 如果油门为0且当前已解锁，则上锁飞控
    else if(control->throttle  == 0 && control->armed)
    {
        FlightControl_Arm(0);  // 上锁
    }
}

