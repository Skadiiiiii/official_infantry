#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"

Can_Export_Data_t Can1_Export_Data;//用来存储接收到的CAN1数据
Can_Export_Data_t Can2_Export_Data;//用来存储接收到的CAN2数据

/**
  * @brief  CAN1过滤器初始化
  */
void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN过滤器编号，范围0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
  sFilterConfig.FilterIdHigh = 0x0000;			//32位下，存储要过滤ID的高16位
  sFilterConfig.FilterIdLow = 0x0000;					//32位下，存储要过滤ID的低16位
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//掩码模式下，存储的是掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//报文通过过滤器的匹配后，存储到哪个FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//激活过滤器
  sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

/**
  * @brief  CAN2过滤器初始化
  */
 void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 14;                       //CAN过滤器编号，范围0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
  sFilterConfig.FilterIdHigh = 0x0000;			//32位下，存储要过滤ID的高16位
  sFilterConfig.FilterIdLow = 0x0000;					//32位下，存储要过滤ID的低16位
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//掩码模式下，存储的是掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//报文通过过滤器的匹配后，存储到哪个FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//激活过滤器
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

/**
  * @brief  处理CAN接收到的数据
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN总线接收回调函数
{
	if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can1_Export_Data.CAN_RxHeader, Can1_Export_Data.CAN_RxMessage); //从CAN总线接收数据
  }
  if (Can1_Export_Data.CAN_RxHeader.StdId >= 0x201 && Can1_Export_Data.CAN_RxHeader.StdId <= 0x204)
  {
		M3508_chassis_getInfo(Can1_Export_Data);
  }
	else if(Can1_Export_Data.CAN_RxHeader.StdId >= 0x205 && Can1_Export_Data.CAN_RxHeader.StdId <= 0x209)
	{
		M3508_shoot_getInfo(Can1_Export_Data);
	}
	
  if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can2_Export_Data.CAN_RxHeader, Can2_Export_Data.CAN_RxMessage); //从CAN总线接收数据
  }
  if (Can2_Export_Data.CAN_RxHeader.StdId == 0x205)
  {
		M6020_Yaw_getInfo(Can2_Export_Data);
	}
	else if (Can2_Export_Data.CAN_RxHeader.StdId == 0x206)
  {
		M6020_Pitch_getInfo(Can2_Export_Data);
	}
}
	
