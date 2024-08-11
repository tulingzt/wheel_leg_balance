#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"

typedef enum
{
    CHASSIS_MODE_PROTECT,

    CHASSIS_MODE_REMOTER_FOLLOW,
    CHASSIS_MODE_REMOTER_ROTATE1,
    CHASSIS_MODE_REMOTER_ROTATE2,

    CHASSIS_MODE_KEYBOARD_FOLLOW,
    CHASSIS_MODE_KEYBOARD_ROTATE,
    CHASSIS_MODE_KEYBOARD_FIGHT,
    CHASSIS_MODE_KEYBOARD_UNFOLLOW,
    CHASSIS_MODE_KEYBOARD_PRONE,  //趴倒模式
} chassis_mode_e;

typedef struct
{
    uint8_t stop_cnt;
    uint8_t reset_flag;
    float last_position;
} motor_reset_t;

typedef struct
{
    float remote, keyboard;
} chassis_scale_t;

typedef struct
{
    float vx, vy;
} chassis_speed_t;

typedef struct
{
    uint8_t init;
    uint8_t joint_motor_reset;
    chassis_mode_e mode;
    float wheel_max;
    chassis_speed_t input, output;
} chassis_t;

extern chassis_t chassis;

void chassis_task(void const *argu);

#endif
