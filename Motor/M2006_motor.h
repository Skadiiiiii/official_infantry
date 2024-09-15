#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"
 
typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
	
	  uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
} M2006s_t;

extern M2006s_t M2006s;

void set_M2006_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void M2006_getInfo(Can_Export_Data_t RxMessage);

#endif
