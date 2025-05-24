#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float kp;        // 比例系数
    float ki;        // 积分系数
    float kd;        // 微分系数
    float error;     // 当前误差
    float prev_error;// 上一次误差
    float integral;  // 积分项
    float derivative;// 微分项
    float output;    // PID输出
    float output_min;// 输出下限
    float output_max;// 输出上限
} PID_t;

// 初始化PID参数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);

// PID计算
float PID_Compute(PID_t *pid, float setpoint, float input);

// 重置PID状态
void PID_Reset(PID_t *pid);

#endif