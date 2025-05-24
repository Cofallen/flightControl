#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "REG52.H"

#define uchar unsigned char
#define uint unsigned int

// 电机引脚定义
sbit MOTOR1 = P1^0;  // 右前电机
sbit MOTOR2 = P1^1;  // 左前电机
sbit MOTOR3 = P1^2;  // 左后电机
sbit MOTOR4 = P1^3;  // 右后电机

// 初始化电机
void Motor_Init(void);

// 设置电机PWM占空比
void Motor_SetSpeed(uchar motor_id, uint speed);

// 紧急停止所有电机
void Motor_EmergencyStop(void);

void Motor_PWM_Update(void);

#endif