 #ifndef __DR16_REMOTE__H__
#define __DR16_REMOTE__H__

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3

typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

typedef enum
{
    RemotePole_UP = 1,  //��
    RemotePole_MID = 3, //��
    RemotePole_DOWM = 2 //��
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //ң������s1��s2����

typedef struct
{
    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //����ֵ��
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //����
    } Robot_TargetValue;  //ң�ؼ����������˶��ٶ�
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־ 
		uint8_t Alldead;
} DR16_Export_Data_t;         //�������ļ�ʹ�õ�������ݡ�

#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}

#define DR16_ExportDataGroundInit \
{                                 \
    {0, 0, 0, 0, 0, 0},           \
    &ControlSwitch,               \
    0,                            \
    0,                            \
}

extern DR16_Export_Data_t DR16_Export_Data;
extern rc_info_t rc;
		
void RemoteControl_Output(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);

#endif

