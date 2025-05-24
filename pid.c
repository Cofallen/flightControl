#include "pid.h"

// 初始化PID参数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->output_min = min;
    pid->output_max = max;
}

// PID计算
float PID_Compute(PID_t *pid, float setpoint, float input)
{
    // 计算误差
    pid->error = setpoint - input;
    
    // 积分项
    pid->integral += pid->error;
    
    // 限制积分项大小，防止积分饱和
    if(pid->integral > 100) pid->integral = 100;
    if(pid->integral < -100) pid->integral = -100;
    
    // 微分项
    pid->derivative = pid->error - pid->prev_error;
    
    // 计算PID输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    
    // 限制输出范围
    if(pid->output > pid->output_max) pid->output = pid->output_max;
    if(pid->output < pid->output_min) pid->output = pid->output_min;
    
    // 保存当前误差作为下一次的前一个误差
    pid->prev_error = pid->error;
    
    return pid->output;
}

// 重置PID状态
void PID_Reset(PID_t *pid)
{
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}