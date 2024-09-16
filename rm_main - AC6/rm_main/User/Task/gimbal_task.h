#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "pid.h"
#include "stdint.h"

typedef struct
{
    pid_t pid;
    float ref, fdb;
} gimbal_pid_t;

typedef struct
{
    float ecd_remote, ecd_keyboard;
    float angle_remote, angle_keyboard;
} gimbal_scale_t;

typedef struct
{
    gimbal_pid_t yaw_angle, yaw_spd;
    gimbal_pid_t pit_angle, pit_spd;
    feed_forward_t yaw_feedforward;
    float yaw_output, pit_output;
    float start_up;//起身标志位
} gimbal_t;

extern gimbal_t gimbal;

void gimbal_task(void const *argu);

#endif
