#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <reg52.h>

#include <intrins.h>

#define uchar unsigned char
#define uint unsigned int

// MPU6050地址
#define MPU6050_ADDR 0xD0

// 寄存器地址
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B

// I2C引脚定义
sbit SCL = P1^6;
sbit SDA = P1^7;

typedef struct MPU6050_Data_t
{
    short acc_x;  // 加速度计X轴数据
    short acc_y;  // 加速度计Y轴数据
    short acc_z;  // 加速度计Z轴数据
    short gyro_x; // 陀螺仪X轴数据
    short gyro_y; // 陀螺仪Y轴数据
    short gyro_z; // 陀螺仪Z轴数据
    short temp;   // 温度数据
    float pitch; // 俯仰角
    float roll;  // 横滚角
    float yaw;   // 偏航角
} MPU6050_Data_t;

// 函数声明
void I2C_Init();
void I2C_Start();
void I2C_Stop();
bit I2C_Write(uchar dat);
uchar I2C_Read(bit ack);
void MPU6050_Init();
void MPU6050_WriteReg(uchar reg_addr, uchar reg_data);
uchar MPU6050_ReadReg(uchar reg_addr);
void MPU6050_ReadData(short *acc_x, short *acc_y, short *acc_z, short *gyro_x, short *gyro_y, short *gyro_z, short *temp);
void UART_Init();
void UART_SendByte(uchar dat);
void UART_SendString(char *s);
void Delay_ms(uint ms);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif