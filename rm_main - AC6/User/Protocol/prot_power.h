#ifndef __PROT_POWER_H
#define __PROT_POWER_H

#include "stdint.h"

typedef struct
{
    float volage, current;          //接收的信息 电容电压 底盘电流
    float min_volage, max_volage;   //电容当前电压 最小电压 最大电压
    uint8_t volume_percent;         //电容电压百分比
    struct
    {
            uint8_t cap_v_over : 1;
            uint8_t cap_v_low : 1;
            uint8_t bat_v_over : 1;
            uint8_t bat_v_low : 1;
            uint8_t cap_i_over : 1;
            uint8_t chassis_i_over : 1;
            uint8_t chassis_msg_miss : 1;
            uint8_t judge_msg_miss : 1;	
    }state;
} supercap_t;

typedef struct
{
    float judge_chassis_power;              //裁判系统反馈的底盘实时功率
    float judge_power_buffer;               //裁判系统反馈的底盘缓冲能量
    float judge_max_power;                  //裁判系统反馈的底盘功率上限
    float min_buffer;                       //当缓存能量低于此值开始功率控制
    float limit_kp;                         //限制比例
    float limit_power;                      //限制功率

    float give_power_wheel[2];
    float scaled_pwoer_wheel[2];
    float total_power_wheel;
    float power_scale;
    uint8_t online;
} power_control_t;

extern supercap_t supercap;
extern power_control_t power_control;

void power_init(void);
void power_judge_update(void);
float motor_power_calcu(float current, float wheel_speed_fdb);
void power_limit_current(void);
float power_limit_speed(void);
void power_get_data(uint8_t *data);

uint8_t power_check_offline(void);

#endif
