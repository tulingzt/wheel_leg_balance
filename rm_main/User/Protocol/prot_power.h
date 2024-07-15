#ifndef __PROT_POWER_H
#define __PROT_POWER_H

#include "stdint.h"

typedef enum
{
    CAP_PROTECT_MODE = 0,  //不充不放模式
    CAP_CHARGE_MODE = 1,   //只充不放模式
    CAP_DISCHARGE_MODE = 2 //边充边放模式
} cap_mode_t;

typedef struct
{
    cap_mode_t mode;                        //电容模式
    float volage, min_volage, max_volage, limit_volage;   //电容当前电压 最小电压 最大电压 限制电流时电压
    uint8_t volume_percent;                 //电容电压百分比
    float charge_power_fdb;
    float charge_power_ref;
    float charge_current_ref;
    
} supercap_t;

typedef struct
{
    float judge_chassis_power;              //裁判系统反馈的底盘实时功率
    float judge_power_buffer;               //裁判系统反馈的底盘缓冲能量
    float judge_max_power;                  //裁判系统反馈的底盘功率上限
    float min_buffer;                       //当缓存能量低于此值开始功率控制
    float limit_kp;                         //限制比例
    
    float source_power;                     //电源功率
    float chassis_power;                    //底盘功率
} power_control_t;

extern supercap_t supercap;
extern power_control_t power_control;

void power_init(void);
void power_judge_update(void);
void supercap_mode_update(void);
void supercap_control(void);
void power_output_data(void);
void power_get_data(uint8_t *data);

#endif
