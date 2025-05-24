#include "mpu6050.h"
#include <reg52.h>
#include <stdio.h>
#include <bluebooth.h>
#include "motor.h"
#include "fc.h"
#include "main.h"
#include "STRING.H"

#define BUFFER_SIZE 35

sbit LED2=P0^2;
sbit LED3=P0^3;
sbit M1=P1^0;
sbit M2=P1^1;
sbit M3=P1^2;
sbit M4=P1^3;

int i;
int roll, pitch, yaw;

MPU6050_Data_t mpu6050_data;


void main()
{
    char buffer[BUFFER_SIZE];  // 增大缓冲区，以适应更多数据

    I2C_Init();
    UART_Init();  // 初始化串口(蓝牙)
    MPU6050_Init();
    M1=0;M2=0;M3=0;M4=0;LED2=0;
    // 蓝牙模块可能需要初始 化时间
    Delay_ms(1000);
    Bluetooth_SendString("MPU6050 Data Ready (with Complementary Filter)\r\n");
    while(1)
    {
        FlightControl_UpdateAttitude(&mpu6050_data);
        // 发送加速度数据
        // sprintf(buffer, "A:%d,%d,%d", mpu6050_data.acc_x, mpu6050_data.acc_y, mpu6050_data.acc_z);
        // sprintf(buffer, "A:%d,%d,%d", acc_x, acc_y, acc_z);
        roll = (int)(mpu6050_data.roll);
        pitch = (int)(mpu6050_data.pitch);
        yaw = (int)(mpu6050_data.yaw);
        sprintf(buffer, "roll: %d, pitch: %d, yaw: %d"
                , roll, pitch, yaw);
        Bluetooth_SendString(buffer);
        
        // // 发送陀螺仪数据
        // sprintf(buffer, "|G:%d,%d,%d", gyro_x, gyro_y, gyro_z);
        // Bluetooth_SendString(buffer);
        
        
        Delay_ms(100); // 100ms刷新一次（dt = 0.1s）
    }
}