#include "mpu6050.h"
#include <reg52.h>
#include "fc.h"
#include "MATH.H"

#define X_ACCEL_OFFSET 0 
#define Y_ACCEL_OFFSET 0 
#define Z_ACCEL_OFFSET 0 

float gyro_offset[3] = {0}; // 陀螺仪Z轴偏置
// I2C初始化
void I2C_Init()
{
    SCL = 1;
    SDA = 1;
}

// I2C起始信号
void I2C_Start()
{
    SDA = 1;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 0;
    _nop_(); _nop_();
    SCL = 0;
}

// I2C停止信号
void I2C_Stop()
{
    SDA = 0;
    SCL = 1;
    _nop_(); _nop_();
    SDA = 1;
    _nop_(); _nop_();
}

// I2C写一个字节
bit I2C_Write(uchar dat)
{
    uchar i;
    bit ack;
    
    for(i=0; i<8; i++)
    {
        SDA = (dat & 0x80) ? 1 : 0;
        SCL = 1;
        _nop_(); _nop_();
        SCL = 0;
        dat <<= 1;
    }
    
    SDA = 1;
    SCL = 1;
    _nop_(); _nop_();
    ack = SDA;
    SCL = 0;
    
    return ack;
}

// I2C读一个字节
uchar I2C_Read(bit ack)
{
    uchar i, dat = 0;
    
    SDA = 1;
    for(i=0; i<8; i++)
    {
        SCL = 1;
        _nop_(); _nop_();
        dat <<= 1;
        if(SDA) dat |= 0x01;
        SCL = 0;
        _nop_(); _nop_();
    }
    
    SDA = ack ? 0 : 1;
    SCL = 1;
    _nop_(); _nop_();
    SCL = 0;
    
    return dat;
}

// MPU6050初始化
void MPU6050_Init()
{
    MPU6050_WriteReg(PWR_MGMT_1, 0x00); // 解除休眠状态
    MPU6050_WriteReg(SMPLRT_DIV, 0x07); // 采样率分频
    MPU6050_WriteReg(CONFIG, 0x06);     // 低通滤波
    MPU6050_WriteReg(GYRO_CONFIG, 0x18); // 陀螺仪±2000dps
    MPU6050_WriteReg(ACCEL_CONFIG, 0x18); // 加速度计±16g
}

// 写MPU6050寄存器
void MPU6050_WriteReg(uchar reg_addr, uchar reg_data)
{
    I2C_Start();
    I2C_Write(MPU6050_ADDR);
    I2C_Write(reg_addr);
    I2C_Write(reg_data);
    I2C_Stop();
}

// 读MPU6050寄存器
uchar MPU6050_ReadReg(uchar reg_addr)
{
    uchar reg_data;
    
    I2C_Start();
    I2C_Write(MPU6050_ADDR);
    I2C_Write(reg_addr);
    
    I2C_Start();
    I2C_Write(MPU6050_ADDR | 0x01);
    reg_data = I2C_Read(0);
    I2C_Stop();
    
    return reg_data;
}

// 读取MPU6050数据
void MPU6050_ReadData(short *acc_x, short *acc_y, short *acc_z, 
                      short *gyro_x, short *gyro_y, short *gyro_z, short *temp)
{
    *acc_x = MPU6050_ReadReg(ACCEL_XOUT_H) << 8 | MPU6050_ReadReg(ACCEL_XOUT_L);
    *acc_y = MPU6050_ReadReg(ACCEL_YOUT_H) << 8 | MPU6050_ReadReg(ACCEL_YOUT_L);
    *acc_z = MPU6050_ReadReg(ACCEL_ZOUT_H) << 8 | MPU6050_ReadReg(ACCEL_ZOUT_L);
    
    *temp = MPU6050_ReadReg(TEMP_OUT_H) << 8 | MPU6050_ReadReg(TEMP_OUT_L);
    
    *gyro_x = MPU6050_ReadReg(GYRO_XOUT_H) << 8 | MPU6050_ReadReg(GYRO_XOUT_L);
    *gyro_y = MPU6050_ReadReg(GYRO_YOUT_H) << 8 | MPU6050_ReadReg(GYRO_YOUT_L);
    *gyro_z = MPU6050_ReadReg(GYRO_ZOUT_H) << 8 | MPU6050_ReadReg(GYRO_ZOUT_L);
}

// 零飘修正
void MPU6050_Calibrate(short *acc_x, short *acc_y, short *acc_z, 
    short *gyro_x, short *gyro_y, short *gyro_z)
{
    *acc_x += X_ACCEL_OFFSET;
    *acc_y += Y_ACCEL_OFFSET;
    *acc_z += Z_ACCEL_OFFSET;
    *gyro_x -= gyro_offset[0];
    *gyro_y -= gyro_offset[1];
    *gyro_z -= gyro_offset[2];
}

// 上电100次求平均去零飘
MPU6050_Data_t mpuCalibrate_data;
void MPU6050_CalibrateInit() {
    float sum[3] = 0.0f;
    int j = 0;
    for (j = 0; j < 100; j++) {
        MPU6050_ReadData(
            &mpuCalibrate_data.acc_x, &mpuCalibrate_data.acc_y, &mpuCalibrate_data.acc_z,
            &mpuCalibrate_data.gyro_x, &mpuCalibrate_data.gyro_y, &mpuCalibrate_data.gyro_z, &mpuCalibrate_data.temp);
        sum[2] += mpuCalibrate_data.gyro_z;
        sum[0] += mpuCalibrate_data.acc_x;
        sum[1] += mpuCalibrate_data.acc_y;
    }
    gyro_offset[2] = sum[2] / 100.0f;
    gyro_offset[0] = sum[0] / 100.0f;
    gyro_offset[1] = sum[1] / 100.0f;
}
// #define Kp 100.0f                        // 比例增益支配率收敛到加速度计/磁强计
// #define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
// #define halfT 0.001f                // 采样周期的一半

// float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
// float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
 
// // float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角
 
// // 原始数据转化欧拉角
// void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
// {
//     float norm;
//     float vx, vy, vz;
//     float ex, ey, ez;  

//     // 测量正常化
//     norm = sqrt(ax*ax + ay*ay + az*az);      
//     ax = ax / norm;                   //单位化
//     ay = ay / norm;
//     az = az / norm;      

//     // 估计方向的重力
//     vx = 2*(q1*q3 - q0*q2);
//     vy = 2*(q0*q1 + q2*q3);
//     vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//     // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
//     ex = (ay*vz - az*vy);
//     ey = (az*vx - ax*vz);
//     ez = (ax*vy - ay*vx);

//     // 积分误差比例积分增益
//     exInt = exInt + ex*Ki;
//     eyInt = eyInt + ey*Ki;
//     ezInt = ezInt + ez*Ki;

//     // 调整后的陀螺仪测量
//     gx = gx + Kp*ex + exInt;
//     gy = gy + Kp*ey + eyInt;
//     gz = gz + Kp*ez + ezInt;

//     // 整合四元数率和正常化
//     q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//     q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//     q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//     q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

//     // 正常化四元
//     norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//     q0 = q0 / norm;
//     q1 = q1 / norm;
//     q2 = q2 / norm;
//     q3 = q3 / norm;

//     fc_attitude.current.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
//     fc_attitude.current.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
//     fc_attitude.current.yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;   //此处没有价值，注掉
// }