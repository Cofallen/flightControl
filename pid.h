#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float kp;        // 比例系数
    float error;     // 当前误差
    float prev_error;// 上一次误差
    float output;     // PID输出
} PID_t;

// 初始化PID参数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);

// PID计算
float PID_Compute(PID_t *pid, float setpoint, float input);

// 重置PID状态
void PID_Reset(PID_t *pid);

#endif