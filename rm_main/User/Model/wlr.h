#ifndef __WHEEL_LEG_ROBOT_H
#define __WHEEL_LEG_ROBOT_H

#include "stdint.h"
#include "pid.h"
#include "kalman_filter.h"

typedef struct
{
    float X_ref[10];
    float X_fdb[10];
    float X_diff[10];
    float U_ref[4];
    float K[4 * 10];
    //leg_w为腿摆角角速度 用来差分计算dot_leg_w腿摆角加速度
    float dot_leg_w[2], last_leg_w[2];
} lqr_t;

//全身运动控制
typedef struct
{
    //宏观控制数据 其它数据像腿摆角、机体倾角默认为0
    float s_ref, v_ref, yaw_ref, wz_ref;
    float high_set;
    //反馈数据
    float s_fdb, v_fdb, yaw_fdb, wz_fdb;
    float roll_fdb, pit_fdb, wy_fdb, az_fdb;
    //中间变量
    float yaw_err, s_adapt;
    //补偿
    float K_adapt;//状态预测补偿系数
    float roll_offs, inertial_offs;//roll补偿 惯性力补偿
    float yaw_offset;//小陀螺偏置
    //期望限制系数
    float K_ref[2];
    //控制标志
    uint8_t jump_flag, jump_cnt, high_flag, prone_flag, ctrl_mode;
    //单侧控制参数
    struct
    {
        //接收数据
        float q1, q4;
        float w1, w4;
        float t1, t4;
        float qy, wy;
        //发送数据
        float T1, T4, Tw;
        float P1, P4;
        //中间数据
        uint8_t fly_flag;
        uint8_t fly_cnt;
        float Fn_kal, Fn_fdb;
        float T0, Fy;
        //状态预测
        float predict_wy, T_adapt;
    } side[2];
} wlr_t;

extern wlr_t wlr;
extern lqr_t lqr;
extern kalman_filter_t kal_3508_vel[2];
extern pid_t pid_leg_length[2];
extern pid_t pid_leg_length_fast[2];
extern pid_t pid_roll;

void wlr_init(void);
void wlr_protest(void);
void wlr_control(void);

#endif
