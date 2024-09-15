
#include "DJI_IMU.h"
#include "INS_task.h"
//����can������֡�����ֽ���Ϊ8
//ŷ���ǵ�һ��������ݾ���һ��float���ͱ�������4���ֽ�
//��Ҳ�����ֻ�ܷ�������
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
IMU_Rec_Data_t DJI_C_IMU;
//���ձ���
IMU_CAL_t IMU_CAL;

extern float INS_angle[3];      //euler angle, unit rad.ŷ���� ��λ rad
extern float INS_gyro[3];

#if send_way == 0
/*��һ�ַ��ͷ�ʽ��������*/
//ŷ����
void Euler_Send_Fun(Euler_Send_u Euler_Send)
{
	
	DJI_C_IMU.yaw = INS_angle[0] * (180/PI) + 180.0f;
	DJI_C_IMU.pitch = INS_angle[2] * (180/PI) + 180.0f;
	DJI_C_IMU.Gyro_z = INS_gyro[2] * (180/PI);
	DJI_C_IMU.Gyro_y = INS_gyro[0] * (180/PI);


  if (DJI_C_IMU.yaw - DJI_C_IMU.last_yaw < -300.0f)
    {
        DJI_C_IMU.turnCounts++;
    }
    if (DJI_C_IMU.last_yaw - DJI_C_IMU.yaw < -300.0f)
    {
        DJI_C_IMU.turnCounts--;
    }
    DJI_C_IMU.total_yaw = DJI_C_IMU.yaw + DJI_C_IMU.turnCounts * 360.0f;

    DJI_C_IMU.last_yaw = DJI_C_IMU.yaw;
		
    //pitch��Ĺ��㴦��
    if (DJI_C_IMU.pitch - DJI_C_IMU.last_pitch < -300.0f)
    {
        DJI_C_IMU.pitch_turnCounts++;
    }
    if (DJI_C_IMU.last_pitch - DJI_C_IMU.pitch < -300.0f)
    {
        DJI_C_IMU.pitch_turnCounts--;
    }
    DJI_C_IMU.total_pitch = DJI_C_IMU.pitch + DJI_C_IMU.pitch_turnCounts * 360.0f;

    DJI_C_IMU.last_pitch = DJI_C_IMU.pitch;
}
//���ٶ�
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send)
{
//	//8��1�ֽڵĻ���ֲ�����
//	uint8_t data[8];
//	
//	//Yaw��angle
//	data[0] = Gyro_Send.Gyro_zy[0];
//	data[1] = Gyro_Send.Gyro_zy[1];
//	data[2] = Gyro_Send.Gyro_zy[2];
//	data[3] = Gyro_Send.Gyro_zy[3];
//	
//	//Pitch��angle
//	data[4] = Gyro_Send.Gyro_zy[4];
//	data[5] = Gyro_Send.Gyro_zy[5];
//	data[6] = Gyro_Send.Gyro_zy[6];
//	data[7] = Gyro_Send.Gyro_zy[7];
	
	//��CANͨѶ����
	//CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

//���ս��
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.rx_header.StdId != IMU_CAL_REIID)
	{
		return;
	}
	//��ȡ�����ǵ�ǰ��У׼״̬
	IMU_CAL.real_Status = CAN_Rx_Structure.rx_data[0];
}
#endif

#if send_way == 1
/*�ڶ��ַ�����ʽ��ָ��*/
//ŷ����
void Euler_Send_Fun(float Yaw,float Pitch)
{
	//����ָ�붼ռ4���ֽ�
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Yaw;
	p[1] = (unsigned char*)&Pitch;
	
	//Yaw��angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch��angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//���ٶ�
void Gyro_Send_Fun(float Gyro_z,float Gyro_y)
{
	//����ָ�붼ռ4���ֽ�
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Gyro_z;
	p[1] = (unsigned char*)&Gyro_y;
	
	//Yaw��angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch��angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

#endif
