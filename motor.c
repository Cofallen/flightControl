
#include "motor.h"

// PWM周期计数器
static uint pwm_counter = 0;
// 每个电机的PWM值（0-1000）
static uint motor1_speed = 0;
static uint motor2_speed = 0;
static uint motor3_speed = 0;
static uint motor4_speed = 0;

// 初始化电机
void Motor_Init(void)
{
    // 初始化电机引脚为输出
    MOTOR1 = 0;
    MOTOR2 = 0;
    MOTOR3 = 0;
    MOTOR4 = 0;
    
    // 设置定时器0用于PWM生成
    TMOD |= 0x01;    // 16位定时器模式
    TH0 = 0xFF;      // 设置较短的溢出时间用于PWM
    TL0 = 0xD0;      // 大约100us的定时
    ET0 = 1;         // 允许定时器0中断
    TR0 = 1;         // 启动定时器0
}

// 设置电机PWM占空比 (speed: 0-1000)
void Motor_SetSpeed(uchar motor_id, uint speed)
{
    // 限制speed范围为0-1000
    if(speed > 1000) speed = 1000;
    
    // 设置对应电机的速度
    switch(motor_id)
    {
        case 1: motor1_speed = speed; break;
        case 2: motor2_speed = speed; break;
        case 3: motor3_speed = speed; break;
        case 4: motor4_speed = speed; break;
        default: break;
    }
}

// 紧急停止所有电机
void Motor_EmergencyStop(void)
{
    motor1_speed = 0;
    motor2_speed = 0;
    motor3_speed = 0;
    motor4_speed = 0;
    
    MOTOR1 = 0;
    MOTOR2 = 0;
    MOTOR3 = 0;
    MOTOR4 = 0;
}

// PWM生成函数，在定时器0中断中调用
void Motor_PWM_Update(void)
{
    // PWM计数器递增
    pwm_counter++;
    if(pwm_counter >= 1000)
    {
        pwm_counter = 0;
    }
    
    // 根据计数器值和设定的速度来控制电机引脚
    MOTOR1 = (pwm_counter < motor1_speed) ? 1 : 0;
    MOTOR2 = (pwm_counter < motor2_speed) ? 1 : 0;
    MOTOR3 = (pwm_counter < motor3_speed) ? 1 : 0;
    MOTOR4 = (pwm_counter < motor4_speed) ? 1 : 0;
}