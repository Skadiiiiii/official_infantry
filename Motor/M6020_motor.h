#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
		float    target_rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
		uint16_t last_rotor_angle;
		int32_t  turn_count;
		float 	 total_angle;
		float 	 target_total_rotor_angle;
	
	  uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
} M6020s_t;

extern M6020s_t M6020s_Yaw;
extern M6020s_t M6020s_Pitch;

void M6020_TargetAngle_init(void);
void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage);
void M6020_Pitch_getInfo(Can_Export_Data_t RxMessage);
void set_M6020_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif
