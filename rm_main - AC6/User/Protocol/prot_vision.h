#ifndef __PROT_VISION_H
#define __PROT_VISION_H

#include "stdint.h"
#include "math.h"

#define VISION_DATA_LEN 23

#define NAN_PROCESS(now, last)      \
    do {                            \
        if (isnan(now)) {           \
            (now) = (last);         \
        } else {                    \
            (last) = (now);         \
        }                           \
    } while (0)

typedef enum
{
    NORMAL = 0,
    TAIL_ERROR = 1,
    REPEAT_ERROR = 2
} vision_rx_status_e;

typedef enum
{
    UNAIMING = 0,
    AIMING = 1,
    FIRST_LOST = 2
} vision_aim_status_e;

typedef struct
{
    uint32_t rx_repeat_cnt;
    vision_rx_status_e rx_status;
    vision_aim_status_e aim_status;
    uint32_t new_frame_flag;
    float target_yaw_angle, target_pit_angle;
    float min_err;
    uint32_t shoot_enable;
    uint8_t online;
    __packed union
    {
        uint8_t buff[VISION_DATA_LEN];
         __packed struct
        {
            float yaw;
            float pit;
            float dis;
            float fire;
            float pos;
            uint8_t empty;
            uint8_t cnt : 6;
            uint8_t ist_flag :1;
            uint8_t aim_flag :1;
            uint8_t eof;
        } data;
    } rx[2];
    __packed union
    {
        uint8_t buff[23];
         __packed struct
        {
            uint8_t sof;
            float imu_pit;
            float imu_yaw;
            float imu_pit_spd;
            float imu_yaw_spd;
            uint8_t vacancy :1;
            uint8_t camp :1;
            uint8_t aiming_mode :3;
            uint8_t shooter_speed :3;
            uint8_t empty;
            uint8_t eof1;
            uint8_t eof2;
            uint16_t empty1;
        } data;
    } tx;
} vision_t;

extern vision_t vision;

void vision_get_data(uint8_t *data);
void vision_output_data(void);
uint8_t vision_check_offline(void);

#endif
