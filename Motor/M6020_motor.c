#include "M6020_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M6020s_t M6020s_Yaw;
M6020s_t M6020s_Pitch;

void M6020_TargetAngle_init()
{
	M6020s_Yaw.target_rotor_angle = 7527;
	M6020s_Pitch.target_rotor_angle = 3500;
	M6020s_Yaw.turn_count = 1;
}

/**
  * @brief  获取云台Yaw轴6020的数据
  */
void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage)
{
    //解包数据，数据格式详见C620电调说明书P33
    M6020s_Yaw.last_rotor_angle = M6020s_Yaw.rotor_angle;
    M6020s_Yaw.rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M6020s_Yaw.rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M6020s_Yaw.torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M6020s_Yaw.temp = RxMessage.CAN_RxMessage[6];

//    if (M6020s_Yaw.rotor_angle - M6020s_Yaw.last_rotor_angle < -4096)
//    {
//        M6020s_Yaw.turn_count++;
//    }

//    if (M6020s_Yaw.last_rotor_angle - M6020s_Yaw.rotor_angle < -4096)
//    {
//        M6020s_Yaw.turn_count--;
//    }

////    M6020s_Yaw.total_angle = M6020s_Yaw.rotor_angle + (8192 * M6020s_Yaw.turn_count);
//			M6020s_Yaw.target_total_rotor_angle = M6020s_Yaw.target_rotor_angle + (8192 * M6020s_Yaw.turn_count);

    //帧率统计，数据更新标志位
    M6020s_Yaw.InfoUpdateFrame++;
    M6020s_Yaw.InfoUpdateFlag = 1;
}

/**
  * @brief  获取云台Pitch轴6020的数据
  */
void M6020_Pitch_getInfo(Can_Export_Data_t RxMessage)
{
    //解包数据，数据格式详见C620电调说明书P33
    M6020s_Pitch.last_rotor_angle = M6020s_Pitch.rotor_angle;
    M6020s_Pitch.rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M6020s_Pitch.rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M6020s_Pitch.torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M6020s_Pitch.temp = RxMessage.CAN_RxMessage[6];

//    if (M6020s_Pitch.rotor_angle - M6020s_Pitch.last_rotor_angle < -6500)
//    {
//        M6020s_Pitch.turn_count++;
//    }

//    if (M6020s_Pitch.last_rotor_angle - M6020s_Pitch.rotor_angle < -6500)
//    {
//        M6020s_Pitch.turn_count--;
//    }

//    M6020s_Pitch.total_angle = M6020s_Pitch.rotor_angle + (8192 * M6020s_Pitch.turn_count);

    //帧率统计，数据更新标志位
    M6020s_Pitch.InfoUpdateFrame++;
    M6020s_Pitch.InfoUpdateFlag = 1;
}

/**
  * @brief  发送云台yaw轴和pitch轴6020的目标电压值
  */
void set_M6020_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//形参表示4个电机的电压值
{
  CAN_TxHeaderTypeDef tx_header_6020;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data_6020[8];//用来存储接发送的CAN帧的数据部分
    
	tx_header_6020.StdId = 0x1ff;
  tx_header_6020.IDE   = CAN_ID_STD;
  tx_header_6020.RTR   = CAN_RTR_DATA;
  tx_header_6020.DLC   = 8;//定义发送格式

  tx_data_6020[0] = (v1>>8)&0xff;//将v1的高8位存储到0
  tx_data_6020[1] =    (v1)&0xff;//将v1的低8位存储到1
  tx_data_6020[2] = (v2>>8)&0xff;
  tx_data_6020[3] =    (v2)&0xff;
  tx_data_6020[4] = (v3>>8)&0xff;
  tx_data_6020[5] =    (v3)&0xff;
  tx_data_6020[6] = (v4>>8)&0xff;
  tx_data_6020[7] =    (v4)&0xff;//定义发送内容
	
  HAL_CAN_AddTxMessage(&hcan2, &tx_header_6020, tx_data_6020,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，  信息内容，  指向邮箱号的指针
}
