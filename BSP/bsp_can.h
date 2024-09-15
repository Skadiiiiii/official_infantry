#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CAN_RxMessage[8];
} Can_Export_Data_t;

typedef struct
{
//	uint8_t CANx;															//指定哪个CAN
	CAN_RxHeaderTypeDef rx_header;			//CAN 接收缓冲区句柄
	uint8_t             rx_data[1];				//存储CAN接收数据 数组
}CAN_Rx_TypeDef;

void CAN_1_Filter_Config(void);
void CAN_2_Filter_Config(void);


#endif
