#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "bsp_flash.h"
#include "INS_task.h"
#include "DJI_IMU.h"

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
//Ҫд�� flash �Ĵ�С��
// 1��head_cail_t + 1�� gimbal_cali_t + 3��imu_cali_t �ṹ������ + 5 �� ���������ֵ�����cail_sensor[i].name[3]+ cail_done(��У׼��־λ)����5��4���ֽ�
//#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(imu_cali_t) + CALI_LIST_LENGHT * 4)

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flash��ȡУ׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flashд��У׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void);

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          �������豸У׼
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static unsigned char cali_gyro_hook(uint32_t *cali, unsigned char cmd);   //gyro device cali function

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static imu_cali_t      gyro_cali;       //gyro cali data

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"GYR"};

//��cail_sensor[i] �� ��У׼���� ����Ϊ ��Ӧ�Ĵ�����������
//cali data address
//static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
//        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
//        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
//        (uint32_t *)&mag_cali};
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
				(uint32_t *)&gyro_cali};
//sizeof ������� ���������ݽṹ��Ĵ�С ����ΪʲôҪ / 4�� 
//����Ŀ��cail_sensor[i]�ṹ�� �е� flash_len ���� ͨ�� ���ֵ��ȷ���� 
//��flash�Ķ�д������32λ����4���ֽڣ�
//static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
//    {
//        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
//        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
				sizeof(imu_cali_t) / 4};
//����������������ָ�볣������ֵ�� void* δ�����ָ������������
//�� void* ���ͣ������������� ����ĺ����Ŀ����ǲ�ͬ����ֵ
//void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_gyro_hook};

//static uint32_t calibrate_systemTick;

/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          У׼������main��������
  * @param[in]      pvParameters: ��
  * @retval         none
  */
//��֮ǰδ����У׼����ô����Ҫִ�д�У׼����
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
		static uint8_t key_last_level;		//key�ϴεĵ�ƽ
	
		uint8_t key_level;				//key�ĵ�ƽ�ź�	
		key_last_level = key_up;	//��ʼ��Ϊδ����
		//��ʼ��key��ƽ
		key_level = HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin);
	
    //��ȡң�����ṹ�����������
    //calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {
				//ң�����Ĳ�ͬ��������Ӧ������ͬ�Ĵ�������У׼
				// �� �� cali_sensor[i].cali_cmd = 1;
        //RC_cmd_to_calibrate();
				key_level = HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin);
				if((key_level == key_down && key_last_level == key_up) || (IMU_CAL.real_Status == begin_calibration && IMU_CAL.last_Status == stop_calibration))
				{
					cali_sensor[0].cali_cmd = 1;
				}
				//������һʱ��key�ĵ�ƽ
				key_last_level = key_level;
				//������һʱ��
				IMU_CAL.last_Status = IMU_CAL.real_Status;
				
        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {		
						//������У׼
            if (cali_sensor[i].cali_cmd)
            {		
								//����У׼�Ĵ����Ӻ�����Ϊ��
                if (cali_sensor[i].cali_hook != NULL)
                {			
										//У׼���̳�����
//										Buzzer.mode = heaps_times;
										//��ͨ��ʵʱ��gyro���ݽ��м���õ�gyro_offsetʵ�����ڼ����Angle��ƫ��ֵ��
										//������Ư��ֵ����cail_sensor[i].buf��У׼�������飬��������д��flash��
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {		
												//ȷ�������������� �� У׼״̬���� ��Ϊ �Ѿ�У׼
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;
												//��������У׼�ı�׼λ��0 
                        cali_sensor[i].cali_cmd = 0;
												//����У׼�������д��flash�й����´ο�������
                        //write
                        cali_data_write();
												//buzzer
                        //������
//												Buzzer.mode = Three_times;
                    }
                }
            }
        }

        //��������ִ��
//        Buzzer_Processing();

        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ʹ��ң������ʼУ׼�����������ǣ���̨������
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;
		
		//��ʼ��ʱ �Ƚ�cail_sensor�ṹ�� ��������У׼�ṹ�� ��һЩȷ����ֵ��ʼ��
		//�� ���� ���������ݽṹ��ĵĴ�С �Լ� ���ݽṹ���ָ�� �Լ� ����������У׼�ĺ����ҹ�  
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (unsigned char(*)(uint32_t *, unsigned char))cali_hook_fun[i];
    }
		//��ȡflash�е����ݣ���֮ǰУ׼�󱣴���flash�е�����
    cali_data_read();
		
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {		
			//�ж��Ѿ�У׼���� 
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
					//�ж�ΪУ׼�����Ӻ�����Ϊ��
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init
								//��ִ��У׼���Ӻ�������У׼���Ĳ��ִ��룬����֮ǰУ׼������flash�е�Ư��ֵ ���� ����ʵ�ʽǶȴ���� Ư��ֵ
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
                //buzzer
                //��һ��
//                Buzzer.mode = One_times;
            }
        }
    }
		//��֮ǰδУ׼�������κδ���
}
/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flash��ȡУ׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        //read the data in flash, 
				//����������32λ����4���ֽڣ����ж�ȡ�ģ�����cali_sensor[i].flash_len�ĳ�ʼ������ͨ��sizeof()������ĳ��ȣ��ֽ�������Ҫ /4
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //���ڴ�Ĵ洢���ֽ�Ϊ��λ���������Ƕ�ȡcali_sensor[i].flash_len���ȵ�32λ����4���ֽڣ������ݣ���ôʵ���϶�ȡ���ֽ�����cali_sensor[i].flash_len * 4 ���Ե�ַҲӦ��������ô��
        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
				//CALI_SENSOR_HEAD_LEGHT ֻ��ȡ1��32λ����4���ֽڣ�������
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flashд��У׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;


    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //copy the data of device calibration data
			//��������洢��cail_sensor[i].flash_buf�е�Ư��������д��flash�Ļ�����������
				memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
				//���������������� �� �������Ƿ���У׼��־λ Ҳ��ֵ�� ������������name�и�ϸ�ڣ�������name[i]
				//����name[3]ֻռ3���ֽ� ��cail_doneҲ��Ҫռ1���ֽڣ������渴�Ƶĳ�����4���ֽ�
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }
		
    //erase the page
		//����flashҳ�� 1ҳ 1������
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
		//��Ҫд��flash�Ļ��������� ���� �������ĳ��� д��flash��
		//֮���� +3 ��Ϊ�˷�ֹ  FLASH_WRITE_BUF_LENGHT�Ǹ����� ����/4 �õ��˳��Ȳ�������+3 �Ϳ����ü������ȱ䳤��������ȫ��д��flaah�����ᶪʧ  
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          �������豸У׼
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static unsigned char cali_gyro_hook(uint32_t *cali, unsigned char cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
		//�Ѿ�У׼����
    if (cmd == CALI_FUNC_CMD_INIT)
    {
				//��ԭ��������洢��cail_sensor[i].flash_buf / cail_sensor_buf[i] ����flash�е�Ư��ֵ��ֱ�Ӹ���gyro_offset[i]ʵ���������������ݼ����Ư��ֵ
				gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        
        return 0;
    }
		//��δУ׼����
    else if (cmd == CALI_FUNC_CMD_ON)
    {
				//У׼ʱ���ʼ��
        static uint16_t count_time = 0;
				//������Ư��ֵ�ļ���
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
				//���ж�һ��У׼��ʱ���Ƿ���ڹ涨ʱ�䣬����������
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            //cali_buzzer_off();
            //gyro_cali_enable_control();
            return 1;
        }
				//���ǾͲ�����������
        else
        {
            //gyro_cali_disable_control(); //disable the remote control to make robot no move
            //imu_start_buzzer();
            
            return 0;
        }
    }

    return 0;
}
