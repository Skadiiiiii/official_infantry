#include "cloud_control.h"
#include "dr16.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "pid.h"


/**
  * @brief  过零处理，计算最小偏差
  */
int ComputeMinOffset(int target, int value) 
{
    int err = target - value;
	
    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}

