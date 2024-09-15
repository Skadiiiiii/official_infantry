#include "chassis_control.h"
#include "dr16.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "pid.h"
#include "math.h"
#include "DJI_IMU.h"

//float Chassis_Vx;
//float Chassis_Vy;
int16_t speed_buff[4];
float VOmega;
uint8_t TwisterFlag;
uint8_t imu_start_flag;

/**
  * @brief  取变量的绝对值
  */
static float abs(float num)
{
		int temp;
		if(num<0) temp=-num;
		else temp=num;
		return temp;
}

/**
  * @brief  麦轮解算
  */
static void MecanumCalculate(float X_Move,float Y_Move,float Yaw ,int16_t *Speed)
{
		float target_speed[4];
    float MaxSpeed = 0.0f;
    float Param = 1.0f;
 
	  target_speed[0] = 	X_Move - Y_Move + Yaw;
		target_speed[1] =    X_Move + Y_Move + Yaw;
		target_speed[2] =   Yaw - X_Move + Y_Move  ;
		target_speed[3]=    Yaw -X_Move - Y_Move  ;	

    for(uint8_t i = 0 ; i<4 ; i ++)
    {
        if(abs(target_speed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(target_speed[i]);
        }
    }

    if (MaxSpeed > 10000)
    {
        Param = (float)10000 / MaxSpeed;
    }

    Speed[0] = target_speed[0] * Param;
    Speed[1] = target_speed[1] * Param;
    Speed[2] = target_speed[2] * Param;
    Speed[3] = target_speed[3] * Param;
}

/**
 * @brief 全向公式
 */
static float* Omnidirectional_Formula(float Vx, float Vy)
{
    float RadRaw = 0.0f;
		static float Chassis[2];
	
    float angle = (M6020s_Yaw.rotor_angle - 7527) / M6020_mAngleRatio; //机械角度偏差
    RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
    //全向移动公式。
    Chassis[0] = Vx * cos(RadRaw) - Vy * sin(RadRaw);
    Chassis[1] = Vy * cos(RadRaw) + Vx * sin(RadRaw);
		return Chassis;
}

//static void Omnidirectional_Formula(float Vx, float Vy)
//{
//    float RadRaw = 0.0f;
//	
//    float angle = (M6020s_Yaw.rotor_angle - 7527) / M6020_mAngleRatio; //机械角度偏差
//    RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
//    //全向移动公式。
//    Chassis_Vx = Vx * cos(RadRaw) - Vy * sin(RadRaw);
//    Chassis_Vy= Vy * cos(RadRaw) + Vx * sin(RadRaw);
//}

/**
 * @brief 获取开机后的陀螺仪数据，并设为基准
 */
static void read_start_imu(void)
{
	uint16_t imu_start_angle;
	if(imu_start_flag == 0)
	{
		for(uint8_t i = 0;i < 10;i++)
		{
			M6020s_Yaw.target_rotor_angle = imu_start_angle;
			imu_start_angle = 22.7527f * DJI_C_IMU.total_yaw + 3432;
			imu_start_flag = 1;
//			DR16_Export_Data.Robot_TargetValue.Omega_Value = 0;
		}
	}
}

/**
  * @brief  底盘跟随模式
  */
static void Ship_ChassisWorkMode_follow(float Vx, float Vy,float delta_yaw,float delta_pitch)
{
	read_start_imu();

	M6020s_Yaw.target_rotor_angle   += delta_yaw;
	M6020s_Pitch.target_rotor_angle += delta_pitch;
	
//	M6020s_Yaw.target_rotor_angle   = (int)M6020s_Yaw.target_rotor_angle % 8191;
//	M6020s_Pitch.target_rotor_angle = (int)M6020s_Pitch.target_rotor_angle % 8191;

	M6020s_Yaw.set_voltage   = pid_CascadeCalc_cloud(&motor_pid_Cas_Yaw, M6020s_Yaw.target_rotor_angle,3432 + 22.7527f * DJI_C_IMU.total_yaw,M6020s_Yaw.rotor_speed);
  M6020s_Pitch.set_voltage = pid_CascadeCalc_cloud(&motor_pid_Cas_Pitch, M6020s_Pitch.target_rotor_angle,M6020s_Pitch.rotor_angle,M6020s_Pitch.rotor_speed);
	
	for(uint8_t i = 0 ; i<4 ; i ++)
	{
			M3508s_chassis[i].set_voltage = pid_CascadeCalc_chassis(&motor_pid_Cas_Chassis[i], 7527,M6020s_Yaw.rotor_angle,M3508s_chassis[i].rotor_speed);
	}
	
//	if(M6020s_Yaw.rotor_angle < 7557 && M6020s_Yaw.rotor_angle > 7497)
//	{
//			motor_pid_Cas_Chassis[0].outer.output=0;
//	}
	
	MecanumCalculate(Vx,Vy,motor_pid_Cas_Chassis[0].outer.output,speed_buff);		
				
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_voltage = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
	}

	set_M6020_voltage(
							M6020s_Yaw.set_voltage, //yaw
							M6020s_Pitch.set_voltage, //pitch
							0, 
							0);

	set_M3508_chassis_voltage(
							M3508s_chassis[0].set_voltage, 
							M3508s_chassis[1].set_voltage, 
							M3508s_chassis[2].set_voltage, 
							M3508s_chassis[3].set_voltage);		
}

/**
  * @brief  底盘扭腰模式
  */
static void Ship_ChassisWorkMode_niuyao(float Vx, float Vy,float delta_yaw,float delta_pitch)
{
//	M6020s_Yaw.target_rotor_angle = (int)M6020s_Yaw.target_rotor_angle % 8191;
//	M6020s_Pitch.target_rotor_angle = (int)M6020s_Pitch.target_rotor_angle % 8191;

	M6020s_Yaw.target_rotor_angle += delta_yaw;
	M6020s_Pitch.target_rotor_angle += delta_pitch;			

	M6020s_Yaw.set_voltage = pid_CascadeCalc_cloud(&motor_pid_Cas_Yaw, M6020s_Yaw.target_rotor_angle,3432 + 22.7527f * DJI_C_IMU.total_yaw,M6020s_Yaw.rotor_speed);

	M6020s_Pitch.set_voltage = pid_CascadeCalc_cloud(&motor_pid_Cas_Pitch, M6020s_Pitch.target_rotor_angle,M6020s_Pitch.rotor_angle,M6020s_Pitch.rotor_speed);
	
	switch(TwisterFlag)
	{	
		 case TwisterFlagStart:
				TwisterFlag = TwisterFlagLeft;
				break;
		 
		 case TwisterFlagLeft:
				VOmega=1000;
				if(M6020s_Yaw.rotor_angle>1384&&M6020s_Yaw.rotor_angle<3432)
				{
					TwisterFlag = TwisterFlagRight;
				}
				break;
				
		 case TwisterFlagRight:
				VOmega=-1000;
				if(M6020s_Yaw.rotor_angle<5479&&M6020s_Yaw.rotor_angle>3431)
				{
					TwisterFlag = TwisterFlagLeft;
				}
				break;
	}
	float* Chassis = Omnidirectional_Formula(Vx,Vy);
	
	MecanumCalculate(Chassis[0],Chassis[1],VOmega,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++) 
	{
			M3508s_chassis[i].set_voltage = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
	}

	set_M6020_voltage( 
							M6020s_Yaw.set_voltage, //yaw
							M6020s_Pitch.set_voltage, //pitch
							0, 
							0);

	set_M3508_chassis_voltage(
							M3508s_chassis[0].set_voltage, 
							M3508s_chassis[1].set_voltage, 
							M3508s_chassis[2].set_voltage, 
							M3508s_chassis[3].set_voltage);
}

/**
  * @brief  机器人失能
  */
static void Robot_control_disable()
{
	M6020s_Yaw.set_voltage = 0; 
	M6020s_Pitch.set_voltage = 0;
	for(uint8_t i = 0;i < 4;i++)
	{
		M3508s_chassis[i].set_voltage = 0; 
		M3508s_shoot[i].set_voltage = 0;
	}
	set_M6020_voltage(0,0,0,0);
	set_M3508_chassis_voltage(0,0,0,0);
}

/**
  * @brief  机器人主控制
  */
void Robot_control (void)        
{
	if(rc.sw1 == 3  && rc.sw2 == 3)
	{
		Ship_ChassisWorkMode_follow(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,-0.05f*DR16_Export_Data.Robot_TargetValue.Omega_Value,0.05f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);
	}
	else if(rc.sw1 == 3  && rc.sw2 == 1)
	{
		Ship_ChassisWorkMode_niuyao(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,-0.05f*DR16_Export_Data.Robot_TargetValue.Omega_Value,0.05f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);
	}
	else
	{
		Robot_control_disable();	
		imu_start_flag = 0;
	}

}

