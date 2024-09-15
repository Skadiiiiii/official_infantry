#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "main.h"

#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define DEG_TO_RAD 0.017453292519943295769236907684886f

#define TwisterFlagStart 0
#define TwisterFlagLeft 1
#define TwisterFlagRight 2

void Robot_control (void);

#endif
