#include "pid.h"

// 初始化PID参数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max)
{
    pid->kp = kp;
    pid->error = 0;
    pid->prev_error = 0;
}

// PID计算
float PID_Compute(PID_t *pid, float setpoint, float input)
{
    // 计算误差
    pid->error = setpoint - input;
  
    // 计算PID输出
    pid->output = pid->kp * pid->error;
    
    // 限制输出范围
    if(pid->output > 300) pid->output = 300;
    if(pid->output < -300) pid->output = -300;
    
    // 保存当前误差作为下一次的前一个误差
    pid->prev_error = pid->error;
    
    return pid->output;
}

// 重置PID状态
void PID_Reset(PID_t *pid)
{
    pid->error = 0;
    pid->prev_error = 0;
    pid->output = 0;
}