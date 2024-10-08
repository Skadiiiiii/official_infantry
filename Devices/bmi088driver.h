


#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

//#include "struct_typedef.h"
#include "main.h"


#pragma pack(push, 1)

typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} bmi088_raw_data_t;

#pragma pack(pop)

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f


enum
{
    BMI088_NO_Error = 0x00,
    BMI088_ACC_PWR_CTRL_Error = 0x01,
    BMI088_ACC_PWR_CONF_Error = 0x02,
    BMI088_ACC_CONF_Error = 0x03,
    BMI088_ACC_SELF_TEST_Error = 0x04,
    BMI088_ACC_RANGE_Error = 0x05,
    BMI088_INT1_IO_CTRL_Error = 0x06,
    BMI088_INT_MAP_DATA_Error = 0x07,
    BMI088_GYRO_RANGE_Error = 0x08,
    BMI088_GYRO_BANDWIDTH_Error = 0x09,
    BMI088_GYRO_LPM1_Error = 0x0A,
    BMI088_GYRO_CTRL_Error = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_Error = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_Error = 0x0D,

    BMI088_SELF_TEST_ACCEL_Error = 0x80,
    BMI088_SELF_TEST_GYRO_Error = 0x40,
    BMI088_NO_Sensor = 0xFF,
};

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

extern uint8_t BMI088_Init(void);
extern unsigned char bmi088_accel_self_test(void);
extern unsigned char bmi088_gyro_self_test(void);
extern unsigned char bmi088_accel_init(void);
extern unsigned char bmi088_gyro_init(void);

extern void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time);
extern void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3]);
extern void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate);
extern void BMI088_Read(float gyro[3], float accel[3], float *temperate);
extern uint32_t get_BMI088_sensor_time(void);
extern float get_BMI088_temperate(void);
extern void get_BMI088_gyro(int16_t gyro[3]);
extern void get_BMI088_accel(float accel[3]);

extern void BMI088_read_gyro_who_am_i(void);
extern uint8_t BMI088_read_accel_who_am_i(void);
void imu_pwm_set(uint16_t pwm);

//#define BMI088_TEMP_FACTOR 0.125f
//#define BMI088_TEMP_OFFSET 23.0f

//#define BMI088_WRITE_ACCEL_REG_NUM  6
//#define BMI088_WRITE_GYRO_REG_NUM   6

//#define BMI088_GYRO_DATA_READY_BIT          0
//#define BMI088_ACCEL_DATA_READY_BIT         1
//#define BMI088_ACCEL_TEMP_DATA_READY_BIT    2

//#define BMI088_LONG_DELAY_TIME      80
//#define BMI088_COM_WAIT_SENSOR_TIME 150


//#define BMI088_ACCEL_IIC_ADDRESSE   (0x18 << 1)
//#define BMI088_GYRO_IIC_ADDRESSE    (0x68 << 1)

//#define BMI088_ACCEL_RANGE_3G
////#define BMI088_ACCEL_RANGE_6G
////#define BMI088_ACCEL_RANGE_12G
////#define BMI088_ACCEL_RANGE_24G

//#define BMI088_GYRO_RANGE_2000
////#define BMI088_GYRO_RANGE_1000
////#define BMI088_GYRO_RANGE_500
////#define BMI088_GYRO_RANGE_250
////#define BMI088_GYRO_RANGE_125


//#define BMI088_ACCEL_3G_SEN     0.0008974358974f
//#define BMI088_ACCEL_6G_SEN     0.00179443359375f
//#define BMI088_ACCEL_12G_SEN    0.0035888671875f
//#define BMI088_ACCEL_24G_SEN    0.007177734375f


//#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
//#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
//#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
//#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
//#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f


//typedef __packed struct BMI088_RAW_DATA
//{
//    uint8_t status;
//    int16_t accel[3];
//    int16_t temp;
//    int16_t gyro[3];
//} bmi088_raw_data_t;

//typedef struct BMI088_REAL_DATA
//{
//    uint8_t status;
//    float accel[3];
//    float temp;
//    float gyro[3];
//    float time;
//} bmi088_real_data_t;


//enum
//{
//    BMI088_NO_ERROR                     = 0x00,
//    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
//    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
//    BMI088_ACC_CONF_ERROR               = 0x03,
//    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
//    BMI088_ACC_RANGE_ERROR              = 0x05,
//    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
//    BMI088_INT_MAP_DATA_ERROR           = 0x07,
//    BMI088_GYRO_RANGE_ERROR             = 0x08,
//    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
//    BMI088_GYRO_LPM1_ERROR              = 0x0A,
//    BMI088_GYRO_CTRL_ERROR              = 0x0B,
//    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
//    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

//    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
//    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
//    BMI088_NO_SENSOR                    = 0xFF,
//};





//extern uint8_t BMI088_init(void);
//extern unsigned char bmi088_accel_self_test(void);
//extern unsigned char bmi088_gyro_self_test(void);
//extern unsigned char bmi088_accel_init(void);
//extern unsigned char bmi088_gyro_init(void);

//extern void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time);
//extern void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3]);
//extern void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate);
//extern void BMI088_read(float gyro[3], float accel[3], float *temperate);
//extern uint32_t get_BMI088_sensor_time(void);
//extern float get_BMI088_temperate(void);
//extern void get_BMI088_gyro(int16_t gyro[3]);
//extern void get_BMI088_accel(float accel[3]);


//extern void BMI088_read_gyro_who_am_i(void);
//extern void BMI088_read_accel_who_am_i(void);

#endif



///*!***************************************************
// * @file: MYGYROData.c
// * @brief:        ?      ?  ???      ? ��    
// * @note:	
// ****************************************************/
// #ifndef __BMIO88DRIVER_H__
//#define __BMIO88DRIVER_H__
//#include "main.h"
//#define BMI088 BMI088
//#define xx 0
//#define yy 1
//#define zz 2

//#define G_Z 0

///*     ?  ?? ?  ?  ??  */

//#define MPU_ACC_CALOFFSET_AUTO
//#define  MPU_GROY_CALOFFSET_AUTO
///*---------------------------------------------*/

//#define XAISN   (3)  //3      
//#define ITEMS  6
//#define MPU9250_FILTER_NUM   4
//#define MPU_CORRECTION_FLASH     0x0800F000        // ����     ? FLASH  ?  SIZE=6*3*4 ? 
//#define GG 9.8															//         ?
///*          څ?  6g */
//#define MPU_ACCE_M_S_2 (24.0 * GG / 32768.0) //  ��   ?  LSB  ?m/(s^2)  GG?   "X"          ? 
//#define MPU_GYRO_TO_DPS (2000 / 32768.0)    //  ��   ?  LSB  ? gegree/s
//#define MPU_MAGN_TO_GS (4800 / 16384.0 / 100.0)            //  ��   ?  LSB  ?Gs
//#define MPU_MAGN_TO_UT (4800 / 16384.0)            //  ��   ?  GS  ?UT
//#define MPU_TEMP_K (0.002995177763f)            //degC/LSB

//typedef struct _accdata
//{
//    short origin[XAISN];  //???
//    float offset[XAISN];      //  ??
//    float offset_max[XAISN];  //  ??   ?
//    float offset_min[XAISN];  //  ??  ��?
//    float calibration[XAISN]; //��??
//    float filter[XAISN];      //    ?   ? ?
//    float m_s_2[XAISN];      //  ?   ��   
//} accdata;
//typedef struct _gyrodata
//{
//    short origin[XAISN];  //???
//    float offset_max[XAISN];  //  ??   ?
//    float offset_min[XAISN];  //  ??  ��?
//    float offset[XAISN];      //  ??
//    float calibration[XAISN]; //��??
//    float filter[XAISN];      //    ?   ? ?
//    float dps[XAISN];         //  ?  
//    float radps[XAISN];       //    ?  
//		/* 2022-01-11    ?   ?  */
//		float last_filter[XAISN];
//} gyrodata;

//struct _mpu
//{
//    accdata acc;
//    gyrodata gyro;

//    float Temperature;
//    uint8_t acc_id, gyro_id;
//    union
//    {
//        int16_t data[3];
//        uint8_t buff[6];
//    } ACC, GYRO;
//    uint8_t temp_originalbuff[2];
//    uint8_t gyro_times;
//    uint8_t acc_times;
//    enum
//    {
//        ReadingACC,
//        ReadingGYRO,
//        IDLE,
//    } state;
//		
//    float pitch;
//    float roll;
//    float yaw;
//    float lastyaw;
//		/*   ?       ?   ?   ?  */
//		float yawsum;
//		/*      ? ?   ?    */
//		uint8_t DynamicOffsetEnable;	//  ?   �� ?��?  ?��
//		int32_t DynamicTmp[3];				//   ?     ? 
//		uint32_t DynamicTimes;				// ��??     
//		uint16_t OffsetCnt;						// ��?    
//		uint16_t OffsetErrorRange;		// ��?  ?��
//		float gyro_z;
//		int16_t yaw_turns;
//};

//typedef struct _mpu mpu;
//extern mpu BMI088;

///**
//  * @brief 	  ?     ?             
//  * @retval      ?         ? ?  ??  
//  *	@note	 data   ?     3  ??   ?     3   ?          ?   ??    
//  * @attention    ??? ? ? ?   2000hz
//  */
//void BMI088_Read_Gyro_Data(void);

///**
//  * @brief 	  ?   ???             
//  * @retval      ?         ? ?  ??  
//  *	@note	 data   ?     3  ??   ?     3   ?          ?   ??    
//  * @attention    ??? ? ? ?   1600hz
//  */
//void BMI088_Read_Acc_Data(void);

///**
//  * @brief 	  ? ?      
//  * @retval      ?         ? ?  ??  
//  *	@note	 data   ?     3  ??   ?     3   ?          ?   ??    
//  * @attention    ??? ? ? ?   1600hz
//  */
//void BMI088_Read_Tmp_Data(void);

///*!***************************************************
// * @file: MYGYROData.c
// * @brief:  ?   ??   ?  ? m/(s^2)
// * @note:	
// ****************************************************/
//void IMU_Read(void);
//#endif

