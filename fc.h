#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

#include "REG52.H"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"

// 姿态角度结构体
typedef struct {
    float roll;      // 横滚角
    float pitch;     // 俯仰角
    float yaw;       // 偏航角
} Attitude_t;

// 飞控数据结构体
typedef struct {
    Attitude_t target;    // 目标姿态
    Attitude_t current;   // 当前姿态
} FC_Attitude_t;

typedef struct {
    PID_t pid_roll;       // 横滚PID
    PID_t pid_pitch;      // 俯仰PID
    PID_t pid_yaw;        // 偏航PID
} FC_PID_t;

typedef struct {
    float throttle;       // 油门值
    unsigned char armed;  // 解锁标志
} FC_Control_t;

// 声明对应的全局变量
extern FC_Attitude_t fc_attitude;
extern FC_PID_t fc_pid;
extern FC_Control_t fc_control;

// 初始化飞控系统
void FlightControl_Init(void);

// 设置目标姿态
void FlightControl_SetTarget(float roll, float pitch, float yaw, float throttle);

// 更新当前姿态（从MPU6050读取）
void FlightControl_UpdateAttitude(MPU6050_Data_t *mpuData);

// 计算电机输出值
void FlightControl_CalculateOutput(void);

// 解锁/上锁
void FlightControl_Arm(unsigned char armed);

// 紧急停止
void FlightControl_EmergencyStop(void);

#endif