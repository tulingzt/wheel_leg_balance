#ifndef __CONTROL_DEF_H
#define __CONTROL_DEF_H

#include "stdint.h"

/*----------------------------- player preference ----------------------------- */
#define KEY_CHASSIS_FIGHT       KB_F
#define KEY_CHASSIS_ROTATE      KB_R
#define KEY_CHASSIS_POWER       KB_SHIFT
#define KEY_CHASSIS_MIDSPEED    KB_CTRL
#define KEY_CHASSIS_LOWSPEED    KB_V
#define KEY_CHASSIS_UNFOLLOW    KB_G
#define KEY_CHASSIS_PRONE       KB_Z
#define KEY_CHASSIS_HEIGHT      KB_C
#define KEY_CHASSIS_HEIGHT2     KB_X
#define KEY_CHASSIS_JUMP        KB_NULL

#define KEY_GIMBAL_TURN_R       KB_E
#define KEY_GIMBAL_TURN_L       KB_Q

#define KEY_VISION              KB_B

/*-----------------------------shoot-----------------------------*/
//拨盘频率
#define TRIGGER_PERIOD      90//ms 90->11Hz 40->25Hz 33->30Hz
#define TRIGGER_PERIOD2     55//40 55->18Hz                   

/*-----------------------------chassis---------------------------*/

#define SUPERCAP_CHAGER_VOLAGE    23.6f
#define SUPERCAP_DISCHAGER_VOLAGE	13.5f //超级电容放电电压下限

#define CHASSIS_YAW_OFFSET  31
#define CHASSIS_YAW_FIGHT   ((CHASSIS_YAW_OFFSET + 8192/4) % 8192)
#define CHASSIS_ROTATE_SPEED 6.28f //rad/s

/*-----------------------------gimbal----------------------------*/

#define GIMBAL_PIT_CENTER_OFFSET    5400
#define GIMBAL_PIT_MAX              6400
#define GIMBAL_PIT_MIN              5000

#endif
