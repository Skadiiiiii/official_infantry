#include "main.h"
#include "chassis_control.h"
#include "cmsis_os.h"
#include "tim.h"
#include "Robot_control_task.h"
#include "dr16.h"

//--����״̬
//Offline_Estimate_Typedef robot_offline_estimate;


/**
  * @brief    ������������
**/
void Robot_Control_Fun(void)
{

	RemoteControl_Output();
	Robot_control();
	
}

///**
//  * @brief          ���������������
//**/
//uint8_t Get_Robot_OfflineState(void)
//{
//    return robot_offline_estimate.offline_state;
//}

/**
  * @brief    ����������������
**/
void Robot_Control(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
	for (;;)
  {
		Robot_Control_Fun();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

