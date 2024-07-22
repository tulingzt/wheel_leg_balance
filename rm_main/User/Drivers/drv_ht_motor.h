#ifndef __DRV_HT_MOTOR_H
#define __DRV_HT_MOTOR_H

#include "can_comm.h"
#include "data_list.h"

#define HT_MOTOR_ID         0x00

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N*m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N*m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f    // N*m 18
#define T_MAX 18.0f

typedef struct
{
    list_t list;
    //电机参数
    can_channel_e can_channel;
    uint32_t can_id;
    uint32_t send_cnt, receive_cnt;
    float err_percent;
    uint8_t online;
    //安装角度补偿
    float zero_point;
    //控制数据
    float p, v, kp, kd, t;
    //反馈数据
    float position, velocity, torque;   //rad rad/s N*m
} ht_motor_t;

extern ht_motor_t joint_motor[4];

void ht_motor_init(ht_motor_t *motor, can_channel_e can_channel, uint32_t id, float zero_point);
void ht_motor_set_control_para(ht_motor_t *motor, float p, float v, float kp, float kd, float t);
void ht_motor_set_control_cmd(ht_motor_t *motor, uint8_t cmd);
void ht_motor_get_data(uint8_t id, uint8_t *data);
void ht_motor_output_data(void);

void ht_motor_output_single_data(ht_motor_t *motor);
uint8_t ht_motor_check_offline(void);

#endif
