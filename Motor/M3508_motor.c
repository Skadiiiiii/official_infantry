#include "M3508_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M3508s_t M3508s_chassis[4];
M3508s_t M3508s_shoot[2];

/**
  * @brief  获取底盘四个3508的数据
  */
void M3508_chassis_getInfo(Can_Export_Data_t RxMessage)
{   
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x201);
    //解包数据，数据格式详见C620电调说明书P33
    M3508s_chassis[StdId].rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M3508s_chassis[StdId].rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M3508s_chassis[StdId].torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M3508s_chassis[StdId].temp = RxMessage.CAN_RxMessage[6];

    //帧率统计，数据更新标志位
    M3508s_chassis[StdId].InfoUpdateFrame++;
    M3508s_chassis[StdId].InfoUpdateFlag = 1;
}

/**
  * @brief  获取摩擦轮四个3508的数据
  */
void M3508_shoot_getInfo(Can_Export_Data_t RxMessage)
{   
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x207);
    //解包数据，数据格式详见C620电调说明书P33
    M3508s_shoot[StdId].rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M3508s_shoot[StdId].rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M3508s_shoot[StdId].torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M3508s_shoot[StdId].temp = RxMessage.CAN_RxMessage[6];

    //帧率统计，数据更新标志位
    M3508s_shoot[StdId].InfoUpdateFrame++;
    M3508s_shoot[StdId].InfoUpdateFlag = 1;
}

/**
  * @brief  发送底盘3508的目标电压值
  */
void set_M3508_chassis_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//形参表示4个电机的电压值
{
	CAN_TxHeaderTypeDef tx_header3508;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data3508[8];//用来存储接发送的CAN帧的数据部分
	
	tx_header3508.StdId = 0x200;
  tx_header3508.IDE   = CAN_ID_STD;
  tx_header3508.RTR   = CAN_RTR_DATA;
  tx_header3508.DLC   = 8;//定义发送格式

  tx_data3508[0] = (v1>>8)&0xff;//将v1的高8位存储到0
  tx_data3508[1] =    (v1)&0xff;//将v1的低8位存储到1
  tx_data3508[2] = (v2>>8)&0xff;
  tx_data3508[3] =    (v2)&0xff;
  tx_data3508[4] = (v3>>8)&0xff;
  tx_data3508[5] =    (v3)&0xff;
  tx_data3508[6] = (v4>>8)&0xff;
  tx_data3508[7] =    (v4)&0xff;//定义发送内容	
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header3508, tx_data3508,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，      信息内容，  指向邮箱号的指针
}

/**
  * @brief  发送摩擦轮3508的目标电压值
  */
void set_M3508_shoot_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//形参表示4个电机的电压值
{
	CAN_TxHeaderTypeDef tx_header3508;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data3508[8];//用来存储接发送的CAN帧的数据部分
	
	tx_header3508.StdId = 0x1ff;
  tx_header3508.IDE   = CAN_ID_STD;
  tx_header3508.RTR   = CAN_RTR_DATA;
  tx_header3508.DLC   = 8;//定义发送格式

  tx_data3508[0] = (v1>>8)&0xff;//将v1的高8位存储到0
  tx_data3508[1] =    (v1)&0xff;//将v1的低8位存储到1
  tx_data3508[2] = (v2>>8)&0xff;
  tx_data3508[3] =    (v2)&0xff;
  tx_data3508[4] = (v3>>8)&0xff;
  tx_data3508[5] =    (v3)&0xff;
  tx_data3508[6] = (v4>>8)&0xff;
  tx_data3508[7] =    (v4)&0xff;//定义发送内容	
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header3508, tx_data3508,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，      信息内容，  指向邮箱号的指针
}


