#include "M2006_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M2006s_t M2006s;

/**
  * @brief  ��ȡ����2006������
  */
void M2006_getInfo(Can_Export_Data_t RxMessage)
{   
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M2006s.rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M2006s.rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M2006s.torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M2006s.temp = RxMessage.CAN_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
    M2006s.InfoUpdateFrame++;
    M2006s.InfoUpdateFlag = 1;
}

/**
  * @brief  ���Ͳ���2006��Ŀ���ѹֵ
  */
void set_M2006_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//�βα�ʾ4������ĵ�ѹֵ
{
	CAN_TxHeaderTypeDef tx_header2006;//�����洢���͵�CAN֡��ͷ����Ϣ������֡ID��֡���͡�֡���ȵ�
  uint8_t             tx_data2006[8];//�����洢�ӷ��͵�CAN֡�����ݲ���
	
	tx_header2006.StdId = 0x1ff;
  tx_header2006.IDE   = CAN_ID_STD;
  tx_header2006.RTR   = CAN_RTR_DATA;
  tx_header2006.DLC   = 8;//���巢�͸�ʽ

  tx_data2006[0] = (v1>>8)&0xff;//��v1�ĸ�8λ�洢��0
  tx_data2006[1] =    (v1)&0xff;//��v1�ĵ�8λ�洢��1
  tx_data2006[2] = (v2>>8)&0xff;
  tx_data2006[3] =    (v2)&0xff;
  tx_data2006[4] = (v3>>8)&0xff;
  tx_data2006[5] =    (v3)&0xff;
  tx_data2006[6] = (v4>>8)&0xff;
  tx_data2006[7] =    (v4)&0xff;//���巢������	
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header2006, tx_data2006,(uint32_t*)CAN_TX_MAILBOX0); //���������ָ��������Ž�CAN֡���͵���Ӧ������
												//CAN, ��Ϣ��ʽ��      ��Ϣ���ݣ�  ָ������ŵ�ָ��
}


