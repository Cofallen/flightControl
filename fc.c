#include "fc.h"
#include "bluebooth.h"
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

#define ACCEL_SCALE  2048.0f   // ±16g → 2048 LSB/g
#define GYRO_SCALE   16.4f     // ±2000dps → 16.4 LSB/(°/s)

float acc_roll, acc_pitch, gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
float base_throttle;

// 飞控数据实例
FC_Attitude_t fc_attitude;
// FC_PID_t fc_pid;
FC_Control_t fc_control;

// 补充函数声明
float Complementary_Filter(float acc_angle, float gyro_rate, float dt, float prev_angle);

// // 初始化飞控系统
// void FlightControl_Init(void)
// {
//     // 初始化MPU6050
//     I2C_Init();
//     MPU6050_Init();
    
//     // 初始化电机
//     Motor_Init();
    
//     // 初始化PID控制器
//     PID_Init(&fc_pid.pid_roll, 3.5, 0.01, 2.0, -300, 300);   // 横滚PID
//     PID_Init(&fc_pid.pid_pitch, 3.5, 0.01, 2.0, -300, 300);  // 俯仰PID
//     PID_Init(&fc_pid.pid_yaw, 4.0, 0.02, 0.0, -300, 300);    // 偏航PID
    
//     // 初始化目标姿态
//     fc_attitude.target.roll = 0.0f;
//     fc_attitude.target.pitch = 0.0f;
//     fc_attitude.target.yaw = 0.0f;
//     fc_control.throttle = 0.0f;
    
//     // 默认上锁状态
//     fc_control.armed = 0;
// }

// // 设置目标姿态
// void FlightControl_SetTarget(float roll, float pitch, float yaw, float throttle)
// {
//     fc_attitude.target.roll = roll;
//     fc_attitude.target.pitch = pitch;
//     fc_attitude.target.yaw = yaw;
//     fc_control.throttle = throttle;
// }


// 更新当前姿态（从MPU6050读取）
void FlightControl_UpdateAttitude(MPU6050_Data_t *mpuData)
{
    float ax, ay, az;
    // 从MPU6050读取数据
    MPU6050_ReadData(&mpuData->acc_x, &mpuData->acc_y, &mpuData->acc_z, &mpuData->gyro_x, &mpuData->gyro_y, &mpuData->gyro_z, &mpuData->temp);
    MPU6050_Calibrate(&mpuData->acc_x, &mpuData->acc_y, &mpuData->acc_z, &mpuData->gyro_x, &mpuData->gyro_y, &mpuData->gyro_z);
    // 转换为物理单位
     ax = mpuData->acc_x / ACCEL_SCALE; // 加速度 (g)
     ay = mpuData->acc_y / ACCEL_SCALE;
     az = mpuData->acc_z / ACCEL_SCALE;

    // 计算俯仰角和横滚角（单位：度）
    mpuData->pitch = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI * 1.01f;
    mpuData->roll  = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI * 1.01f;
    mpuData->yaw  += mpuData->gyro_z / GYRO_SCALE * 0.1f * 1.607f; // 偏航角（单位：度）
    // 保持角度在 -180° 到 +180° 范围内
    if (mpuData->yaw > 180.0f) mpuData->yaw -= 360.0f;
    else if (mpuData->yaw < -180.0f) mpuData->yaw += 360.0f;
    
    #ifdef MULTIPLY_INTERVAL
    // 请把上面的更新欧拉角的参数对应换成acc_pitch和acc_roll 把mpuData->yaw注释
    // 目前融合起来 飘的严重
    // 根据陀螺仪计算角速度
    static float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;
    gyro_roll_rate = mpuData->gyro_x / GYRO_SCALE;   // 根据量程±2000dps转换
    gyro_pitch_rate = mpuData->gyro_y / GYRO_SCALE;
    gyro_yaw_rate = mpuData->gyro_z / GYRO_SCALE;
    // // IMUupdate(gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate, ax, ay, az);
    // 使用互补滤波融合数据
    mpuData->roll = Complementary_Filter(acc_roll, gyro_roll_rate, 0.1, prev_roll);
    mpuData->pitch = Complementary_Filter(acc_pitch, gyro_pitch_rate, 0.1, prev_pitch);
    mpuData->yaw += gyro_yaw_rate * 0.1f * 1.44f;  // 偏航角主要依靠陀螺仪积分
    // 更新前一次的角度值
    prev_roll = mpuData->roll;
    prev_pitch = mpuData->pitch;
    prev_yaw = mpuData->yaw;
    #endif // DEBUG
}

// 互补滤波器
float Complementary_Filter(float acc_angle, float gyro_rate, float dt, float prev_angle)
{
    float alpha = 0.98f;
    return (alpha * (prev_angle + gyro_rate * dt) + (1.0f - alpha) * acc_angle);
}

// // 计算电机输出值
// void FlightControl_CalculateOutput(void)
// {
//     float roll_output, pitch_output, yaw_output;
//     uint motor1_output, motor2_output, motor3_output, motor4_output;
    
//     // 如果未解锁，电机输出为0
//     if(!fc_control.armed)
//     {
//         Motor_SetSpeed(1, 0);
//         Motor_SetSpeed(2, 0);
//         Motor_SetSpeed(3, 0);
//         Motor_SetSpeed(4, 0);
//         return;
//     }
    
//     // 计算PID输出
//     roll_output = PID_Compute(&fc_pid.pid_roll, fc_attitude.target.roll, fc_attitude.current.roll);
//     pitch_output = PID_Compute(&fc_pid.pid_pitch, fc_attitude.target.pitch, fc_attitude.current.pitch);
//     yaw_output = PID_Compute(&fc_pid.pid_yaw, fc_attitude.target.yaw, fc_attitude.current.yaw);
    
//     // 基础油门值
//     base_throttle = fc_control.throttle * 10.0f;  // 假设油门范围0-100，转换到0-1000
    
//     // 计算每个电机的输出
//     // M1: 右前 (+pitch, -roll, -yaw)
//     motor1_output = (uint)(base_throttle + pitch_output - roll_output - yaw_output);
    
//     // M2: 左前 (+pitch, +roll, +yaw)
//     motor2_output = (uint)(base_throttle + pitch_output + roll_output + yaw_output);
    
//     // M3: 左后 (-pitch, +roll, -yaw)
//     motor3_output = (uint)(base_throttle - pitch_output + roll_output - yaw_output);
    
//     // M4: 右后 (-pitch, -roll, +yaw)
//     motor4_output = (uint)(base_throttle - pitch_output - roll_output + yaw_output);
    
//     // 确保输出值在合理范围内
//     if(motor1_output < 0) motor1_output = 0;
//     if(motor2_output < 0) motor2_output = 0;
//     if(motor3_output < 0) motor3_output = 0;
//     if(motor4_output < 0) motor4_output = 0;
    
//     if(motor1_output > 1000) motor1_output = 1000;
//     if(motor2_output > 1000) motor2_output = 1000;
//     if(motor3_output > 1000) motor3_output = 1000;
//     if(motor4_output > 1000) motor4_output = 1000;
    
//     // 设置电机速度
//     Motor_SetSpeed(1, motor1_output);
//     Motor_SetSpeed(2, motor2_output);
//     Motor_SetSpeed(3, motor3_output);
//     Motor_SetSpeed(4, motor4_output);
    
//     // 通过蓝牙发送状态信息
//     // sprintf(buffer, "R:%.1f P:%.1f Y:%.1f T:%.1f\r\n", 
//     //         fc.current.roll, fc.current.pitch, fc.current.yaw, fc.throttle);
//     // Bluetooth_SendString(buffer);
// }

// // 解锁/上锁
// void FlightControl_Arm(unsigned char armed)
// {
//     fc_control.armed = armed;
//     if(!armed)
//     {
//         Motor_SetSpeed(1, 0);
//         Motor_SetSpeed(2, 0);
//         Motor_SetSpeed(3, 0);
//         Motor_SetSpeed(4, 0);
//     }
// }

// // 紧急停止
// void FlightControl_EmergencyStop(void)
// {
//     fc_control.armed = 0;
//     Motor_EmergencyStop();
    
//     // 重置PID状态
//     PID_Reset(&fc_pid.pid_roll);
//     PID_Reset(&fc_pid.pid_pitch);
//     PID_Reset(&fc_pid.pid_yaw);
// }
