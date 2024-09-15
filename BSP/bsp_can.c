#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"

Can_Export_Data_t Can1_Export_Data;//�����洢���յ���CAN1����
Can_Export_Data_t Can2_Export_Data;//�����洢���յ���CAN2����

/**
  * @brief  CAN1��������ʼ��
  */
void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
  sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

/**
  * @brief  CAN2��������ʼ��
  */
 void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 14;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

/**
  * @brief  ����CAN���յ�������
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN���߽��ջص�����
{
	if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can1_Export_Data.CAN_RxHeader, Can1_Export_Data.CAN_RxMessage); //��CAN���߽�������
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
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can2_Export_Data.CAN_RxHeader, Can2_Export_Data.CAN_RxMessage); //��CAN���߽�������
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
	
