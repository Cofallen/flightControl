#include "bluebooth.h"
#include "fc.h"

// ���ڳ�ʼ��(����ͨ��)
void UART_Init()
{
    TMOD &= 0x0F;
    TMOD |= 0x20;    // ��ʱ��1������ʽ2
    TH1 = 0xFD;     // ������9600
    TL1 = 0xFD;
    TR1 = 1;        // ������ʱ��1
    
    SCON = 0x50;    // ���ڹ�����ʽ1���������
    ES = 1;         // �������ж�
    EA = 1;         // �����ж�
}

// ����һ���ֽ�
void Bluetooth_SendByte(uchar dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// �����ַ���
void Bluetooth_SendString(char *s)
{
    while(*s)
    {
        Bluetooth_SendByte(*s++);
    }
}


// static int throttle = 0;    
#define STEP_THROTTLE 50     // ���ŵ�������
#define STEP_ATTITUDE 5      // ��̬��������
// ����������� - ʹ���βδ���fc_attitude
void ProcessCommand(char cmd, FC_Attitude_t *attitude, FC_Control_t *control)
{
    switch(cmd)
    {
        case 'G':  // ���żӴ�
            if(control->throttle  <= 950) control->throttle  += STEP_THROTTLE;
            if(control->throttle  > 1000) control->throttle  = 1000;
            break;
            
        case 'K':  // ���ż�С
            if(control->throttle  >= 50) control->throttle  -= STEP_THROTTLE;
            else control->throttle  = 0;
            break;
            
        case 'H':  // ����(����ƫ����)
            attitude->target.yaw += STEP_ATTITUDE;
            if(attitude->target.yaw > 180.0f) attitude->target.yaw -= 360.0f;
            break;
            
        case 'J':  // ����(��Сƫ����)
            attitude->target.yaw -= STEP_ATTITUDE;
            if(attitude->target.yaw < -180.0f) attitude->target.yaw += 360.0f;
            break;
            
        case 'B':  // ǰ��(��С������)
            attitude->target.pitch = -STEP_ATTITUDE;
            break;
            
        case 'E':  // ����(���Ӹ�����)
            attitude->target.pitch = STEP_ATTITUDE;
            break;
            
        case 'A':  // ������б(��С�����)
            attitude->target.roll = -STEP_ATTITUDE;
            break;
            
        case 'D':  // ������б(���Ӻ����)
            attitude->target.roll = STEP_ATTITUDE;
            break;
            
        case 'S':  // ֹͣ/�ָ�ˮƽ
            attitude->target.roll = 0.0f;
            attitude->target.pitch = 0.0f;
            break;
            
        case 'I':  // ����ֹͣ
        control->throttle  = 0;
            attitude->target.roll = 0.0f;
            attitude->target.pitch = 0.0f;
            attitude->target.yaw = 0.0f;
            FlightControl_EmergencyStop();
            break;
            
        default:
            break;
    }
    
    // ���·ɿ�Ŀ������
    // control->throttle = throttle;
    
    // ������Ŵ�����Сֵ�ҵ�ǰδ������������ɿ�
    if(control->throttle  > 100 && !control->armed)
    {
        FlightControl_Arm(1);  // ����
    }
    // �������Ϊ0�ҵ�ǰ�ѽ������������ɿ�
    else if(control->throttle  == 0 && control->armed)
    {
        FlightControl_Arm(0);  // ����
    }
}

