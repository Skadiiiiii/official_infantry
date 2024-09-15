#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "main.h"
//#include "bsp_buzzer.h"

#define key_down 0
#define key_up 1

//get stm32 chip temperature, to calc imu control temperature.��ȡstm32Ƭ���¶ȣ�����imu�Ŀ����¶�
#define cali_get_mcu_temperature()  get_temprate()    

#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash ��ȡ����
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash д�뺯��
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash��������

// calc the zero drift function of gyro, ������������Ư
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, ������INS task�ڵ���������Ư
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,�����flashҳ��ַ

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,��������ǿ����¶�

#define CALI_FUNC_CMD_ON        1                   //need calibrate,����У׼
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.�Ѿ�У׼��������У׼ֵ

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms ϵͳ��ʱ

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated

#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,������У׼ʱ��

//cali device name
typedef enum
{
    CALI_GYRO = 0,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght				//ð�ţ����� �Ǳ�ʾ�Ҷ����˸� 8��λ�� flash_len ���� ������ֻ�õ� ���� 7��λ
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    unsigned char (*cali_hook)(uint32_t *point, unsigned char cmd);   //cali function
	
} cali_sensor_t;
extern cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

//gyro, accel, mag device
typedef struct
{
    float offset[3]; //x,y,z
    float scale[3];  //x,y,z
} imu_cali_t;

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
extern void cali_param_init(void);

/**
  * @brief          get imu control temperature, unit ��
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          ��ȡimu�����¶�, ��λ��
  * @param[in]      none
  * @retval         imu�����¶�
  */
extern int8_t get_control_temperature(void);
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
extern void calibrate_task(void const *pvParameters);

#endif
