#include "pid.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "cloud_control.h"

pid_struct_t motor_pid_chassis[4];
pid_Cascade_t motor_pid_Cas_Chassis[4];
pid_Cascade_t motor_pid_Cas_Yaw;
pid_Cascade_t motor_pid_Cas_Pitch;

/**
  * @brief PID参数初始化
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

/**
  * @brief  PID速度环计算
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

/**
  * @brief  云台6020速度环PID计算
  */
float pid_calc_cloud(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;

	pid->err[0] = ComputeMinOffset(pid->tar,pid->now);//过零处理
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	pid->err[1] = pid->err[0];
  return pid->output;
}

/**
  * @brief  底盘3508速度位置环PID计算
  */
float pid_CascadeCalc_chassis(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//外层角度
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  云台6020速度位置环PID计算
  */
float pid_CascadeCalc_cloud(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//外层角度
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  电机PID参数初始化
  */
void motor_pid_init()
{
	pid_init(&motor_pid_Cas_Yaw.inner,  300,0.01f,0, 30000, 30000);//init pid 内环速度, kp=300	, ki=0.01, kd=0, output limit = 30000
	pid_init(&motor_pid_Cas_Yaw.outer,  0.18f, 0, 0, 30000, 30000);//init pid 外环角度, kp=0.18, ki=0, 	 kd=0, output limit = 30000
	
	pid_init(&motor_pid_Cas_Pitch.inner,  3, 0, 0, 30000, 30000);//init pid 内环速度, kp=40, ki=3, kd=0, output limit = 30000
	pid_init(&motor_pid_Cas_Pitch.outer,  8, 0, 0, 30000, 30000);//init pid 外环角度, kp=40, ki=3, kd=0, output limit = 30000
	
	for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&motor_pid_Cas_Chassis[i].inner,  500, 0.01f, 0, 30000, 10000);//init pid 内环速度, kp=40, ki=3, kd=0, output limit = 30000
		pid_init(&motor_pid_Cas_Chassis[i].outer,  2.5f, 0, 0, 30000, 10000);//init pid 外环角度, kp=40, ki=3, kd=0, output limit = 30000
  }
	
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_chassis[i], 9.5f, 0.3f ,0, 30000, 15000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	}
}

