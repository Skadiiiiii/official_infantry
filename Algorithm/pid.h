 #ifndef _PID_H
#define _PID_H

#include "main.h"

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float tar;      
  float now;      
  float err[3];   //����������һ�����

  float p_out;
  float i_out;
  float d_out;
  float output;
	
	float delta_u;						//��������ֵ
	float delta_out;					//��������ʽ��� = ������������һ������
	float last_delta_out;
}pid_struct_t;

typedef struct
{
	pid_struct_t outer;
	pid_struct_t inner;
	float output;
}pid_Cascade_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
												
extern pid_struct_t motor_pid_chassis[4];
extern pid_Cascade_t motor_pid_Cas_Chassis[4];
extern pid_Cascade_t motor_pid_Cas_Yaw;
extern pid_Cascade_t motor_pid_Cas_Pitch;
							
float pid_calc(pid_struct_t *pid, float tar, float now);
float pid_calc_cloud(pid_struct_t *pid, float tar, float now);
float pid_CascadeCalc_chassis(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow);
float pid_CascadeCalc_cloud(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow);
void motor_pid_init(void);
	
#endif
