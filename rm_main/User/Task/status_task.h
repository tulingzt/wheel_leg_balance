#ifndef __STATUS_TASK_H
#define __STATUS_TASK_H

#include "stdint.h"

typedef __packed struct {
    uint16_t remote     :1;
    uint16_t vision     :1;
    uint16_t judge      :1;
    uint16_t power      :1;
    uint16_t imu        :3;// 0-2 3
    uint16_t dji_motor  :4;// 0-7 8
    uint16_t ht_motor   :4;// 0-4 5
    uint16_t all        :1;
    __packed struct
    {
        uint16_t comm:1;
        uint16_t gimbal:1;
        uint16_t chassis:1;
        uint16_t shoot:1;
        uint16_t mode_switch:1;
        uint16_t ui:1;
        uint16_t status:1;
        uint16_t all:9;
    } task;
} status_t;

extern status_t status;

void status_task(void const* argument);

#endif
