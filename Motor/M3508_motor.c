#include "M3508_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M3508s_t M3508s_chassis[4];
M3508s_t M3508s_shoot[2];

/**
  * @brief  ��ȡ�����ĸ�3508������
  */
void M3508_chassis_getInfo(Can_Export_Data_t RxMessage)
{   
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x201);
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M3508s_chassis[StdId].rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M3508s_chassis[StdId].rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M3508s_chassis[StdId].torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M3508s_chassis[StdId].temp = RxMessage.CAN_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
    M3508s_chassis[StdId].InfoUpdateFrame++;
    M3508s_chassis[StdId].InfoUpdateFlag = 1;
}

/**
  * @brief  ��ȡĦ�����ĸ�3508������
  */
void M3508_shoot_getInfo(Can_Export_Data_t RxMessage)
{   
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x207);
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M3508s_shoot[StdId].rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M3508s_shoot[StdId].rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M3508s_shoot[StdId].torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M3508s_shoot[StdId].temp = RxMessage.CAN_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
    M3508s_shoot[StdId].InfoUpdateFrame++;
    M3508s_shoot[StdId].InfoUpdateFlag = 1;
}

/**
  * @brief  ���͵���3508��Ŀ���ѹֵ
  */
void set_M3508_chassis_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//�βα�ʾ4������ĵ�ѹֵ
{
	CAN_TxHeaderTypeDef tx_header3508;//�����洢���͵�CAN֡��ͷ����Ϣ������֡ID��֡���͡�֡���ȵ�
  uint8_t             tx_data3508[8];//�����洢�ӷ��͵�CAN֡�����ݲ���
	
	tx_header3508.StdId = 0x200;
  tx_header3508.IDE   = CAN_ID_STD;
  tx_header3508.RTR   = CAN_RTR_DATA;
  tx_header3508.DLC   = 8;//���巢�͸�ʽ

  tx_data3508[0] = (v1>>8)&0xff;//��v1�ĸ�8λ�洢��0
  tx_data3508[1] =    (v1)&0xff;//��v1�ĵ�8λ�洢��1
  tx_data3508[2] = (v2>>8)&0xff;
  tx_data3508[3] =    (v2)&0xff;
  tx_data3508[4] = (v3>>8)&0xff;
  tx_data3508[5] =    (v3)&0xff;
  tx_data3508[6] = (v4>>8)&0xff;
  tx_data3508[7] =    (v4)&0xff;//���巢������	
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header3508, tx_data3508,(uint32_t*)CAN_TX_MAILBOX0); //���������ָ��������Ž�CAN֡���͵���Ӧ������
												//CAN, ��Ϣ��ʽ��      ��Ϣ���ݣ�  ָ������ŵ�ָ��
}

/**
  * @brief  ����Ħ����3508��Ŀ���ѹֵ
  */
void set_M3508_shoot_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//�βα�ʾ4������ĵ�ѹֵ
{
	CAN_TxHeaderTypeDef tx_header3508;//�����洢���͵�CAN֡��ͷ����Ϣ������֡ID��֡���͡�֡���ȵ�
  uint8_t             tx_data3508[8];//�����洢�ӷ��͵�CAN֡�����ݲ���
	
	tx_header3508.StdId = 0x1ff;
  tx_header3508.IDE   = CAN_ID_STD;
  tx_header3508.RTR   = CAN_RTR_DATA;
  tx_header3508.DLC   = 8;//���巢�͸�ʽ

  tx_data3508[0] = (v1>>8)&0xff;//��v1�ĸ�8λ�洢��0
  tx_data3508[1] =    (v1)&0xff;//��v1�ĵ�8λ�洢��1
  tx_data3508[2] = (v2>>8)&0xff;
  tx_data3508[3] =    (v2)&0xff;
  tx_data3508[4] = (v3>>8)&0xff;
  tx_data3508[5] =    (v3)&0xff;
  tx_data3508[6] = (v4>>8)&0xff;
  tx_data3508[7] =    (v4)&0xff;//���巢������	
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header3508, tx_data3508,(uint32_t*)CAN_TX_MAILBOX0); //���������ָ��������Ž�CAN֡���͵���Ӧ������
												//CAN, ��Ϣ��ʽ��      ��Ϣ���ݣ�  ָ������ŵ�ָ��
}


