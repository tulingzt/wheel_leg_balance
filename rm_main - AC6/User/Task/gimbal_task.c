#include "mode_switch_task.h"
#include "status_task.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "prot_vision.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "math_lib.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "string.h"
#include "func_generator.h"
#include "gimbal_task.h"
#include "status_task.h"

FGT_agl_t yaw_test = {
    .Td = 1,
    .time = 0,
    .T1 = 150,
    .T2 = 0,
    .T1_out = 0,
    .T2_out = 0.175,
    .out = 0
};

gimbal_scale_t gimbal_scale = {
    .ecd_remote = 1,
    .ecd_keyboard = 1,
    .angle_remote = 0.00002f,
    .angle_keyboard = 0.00006f
};
gimbal_t gimbal;

static void gimbal_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));

//    pid_init(&gimbal.pit_angle.pid, NONE, 18, 0, 0, 0, 7);//20 0 0 有测速模块
//    pid_init(&gimbal.pit_spd.pid, NONE, -1.2f, -0.006f, 0, 0.4f, 2.2f);//-1.0 -0.008
//    
        pid_init(&gimbal.pit_angle.pid, NONE, 2, 0, 0, 0, 7);//20 0 0 有测速模块
    pid_init(&gimbal.pit_spd.pid, NONE, -0.5, -0.001f, 0, 0.4f, 2.2f);//-1.0 -0.008
    
    pid_init(&gimbal.yaw_angle.pid, NONE, 6, 0, 0, 0, 30);
    pid_init(&gimbal.yaw_spd.pid, NONE, 3.0f, 0.02f, 0, 1.0f, 2.2f);

}

static void gimbal_pid_calc(void)
{
    float yaw_err, pit_max, pit_min;
    //位置环反馈 陀螺仪 -0.6 0.4
    //速度环反馈 陀螺仪
    //此yaw_err用于云台pit限幅
    yaw_err = circle_error(CHASSIS_YAW_OFFSET / 8192.0f * 2 * PI, yaw_motor.ecd / 8192.0f * 2 * PI, 2 * PI);
    pit_max = -arm_cos_f32(yaw_err) * chassis_imu.pit + 0.35f;
    pit_min = -arm_cos_f32(yaw_err) * chassis_imu.pit - 0.55f;
    data_limit(&gimbal.pit_angle.ref, pit_min, pit_max);
    gimbal.pit_angle.fdb = -gimbal_imu.pit;
    gimbal.pit_spd.ref = pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.ref, gimbal.pit_angle.fdb);
//    gimbal.pit_spd.fdb = -gimbal_imu.wy - arm_cos_f32(yaw_err) * chassis_imu.wy;
    gimbal.pit_spd.fdb = -gimbal_imu.wy;
    gimbal.pit_output = pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.ref, gimbal.pit_spd.fdb);

    //pid参数选择
    if ((rc.mouse.r || rc_fsm_check(RC_LEFT_LD) || rc_fsm_check(RC_RIGHT_LD)) && vision.aim_status == AIMING) {
        gimbal.yaw_angle.pid.kp = 45;
        gimbal.yaw_angle.pid.kd = 650;
        gimbal.yaw_spd.pid.kp = 1.4f;
        gimbal.yaw_spd.pid.ki = 0.0f;
        gimbal.yaw_spd.pid.i_out = 0.0f;
        gimbal.yaw_angle.pid.out_max = 15;
    } else {
        gimbal.yaw_angle.pid.kp = 6;
        gimbal.yaw_angle.pid.kd = 0;
        gimbal.yaw_spd.pid.kp = 3.0f;//4.5
        gimbal.yaw_spd.pid.ki = 0.02f;//0.04
        gimbal.yaw_angle.pid.out_max = 15;
    }
    
    if (gimbal.yaw_angle.ref < 0) {
        gimbal.yaw_angle.ref += 2 * PI;
    } else if (gimbal.yaw_angle.ref > 2 * PI) {
        gimbal.yaw_angle.ref -= 2 * PI;
    }
    //视觉测试
//    gimbal.yaw_angle.ref = FGT_agl_calc(&yaw_test);
    gimbal.yaw_angle.fdb = gimbal_imu.yaw;
    //此yaw_err用于云台yaw环形控制
    yaw_err = circle_error(gimbal.yaw_angle.ref, gimbal.yaw_angle.fdb, 2*PI);
    if (gimbal.start_up == 0 && yaw_err < 0.03f)
        gimbal.start_up = 1;

    gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb + yaw_err, gimbal.yaw_angle.fdb);    
//    gimbal.yaw_spd.fdb = gimbal_imu.wz + 0.4f * chassis_imu.wz;
    gimbal.yaw_spd.fdb = gimbal_imu.wz;
    gimbal.yaw_output = pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.ref, gimbal.yaw_spd.fdb);
}

static void gimbal_data_output(void)
{
    dji_motor_set_torque(&pit_motor, gimbal.pit_output);
    dji_motor_set_torque(&yaw_motor, gimbal.yaw_output);
}

static void gimbal_get_vision_data(void)
{
    switch (vision.aim_status) {
        case AIMING: {//识别到目标
            if (vision.new_frame_flag) {
                vision.new_frame_flag = 0;
                gimbal.pit_angle.ref = vision.target_pit_angle;
                gimbal.yaw_angle.ref = vision.target_yaw_angle;
            }
            break;
        }
        case FIRST_LOST: {//首次丢失
            vision.aim_status = UNAIMING;
            gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
            gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
            break;
        }
        case UNAIMING: {//未识别到目标
            if (ctrl_mode == REMOTER_MODE) {
                gimbal.pit_angle.ref -= rc.ch2 * gimbal_scale.angle_remote;
                gimbal.yaw_angle.ref -= rc.ch1 * gimbal_scale.angle_remote;
            } else if (ctrl_mode == KEYBOARD_MODE) {
                gimbal.pit_angle.ref += rc.mouse.y * gimbal_scale.angle_keyboard * 0.5f;
                gimbal.yaw_angle.ref -= rc.mouse.x * gimbal_scale.angle_keyboard;
            }
            break;
        }
        default: break;
    }
}

void gimbal_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    gimbal_init();
    for(;;) {
        thread_wake_time = osKernelSysTick();
//        taskENTER_CRITICAL();
        switch (ctrl_mode) {
            case PROTECT_MODE: {
                gimbal.start_up = 0;//保护模式下，起身标志位置零
//                gimbal.yaw_angle.ref = gimbal_imu.yaw;
                gimbal.yaw_angle.ref = gimbal_imu.yaw + (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI - (float)yaw_motor.ecd / 8192 * 2 * PI;//yaw轴反馈值+电机与前方灯条差值
                gimbal.pit_angle.ref = 0;
                gimbal.pit_output = 0;
                gimbal.yaw_output = 0;
                break;
            }
            case REMOTER_MODE: {
                if (rc_fsm_check(RC_LEFT_LD) || rc_fsm_check(RC_RIGHT_LD)) { //遥控器开启视觉
                    gimbal_get_vision_data();
                } else {
                    gimbal.pit_angle.ref -= rc.ch2 * gimbal_scale.angle_remote;
                    gimbal.yaw_angle.ref -= rc.ch1 * gimbal_scale.angle_remote;
                }
                gimbal_pid_calc();
                break;
            }
            case KEYBOARD_MODE: {
                if (rc.mouse.r == 1) {
                    gimbal_get_vision_data();
                } else {
                    //一键调头
                    if(key_scan_clear(KEY_GIMBAL_TURN_R)) {
                        gimbal.yaw_angle.ref -= PI/2;
                    } 
                    else if (key_scan_clear(KEY_GIMBAL_TURN_L)) {
                        gimbal.yaw_angle.ref += PI/2;
                    }
                    gimbal.pit_angle.ref += rc.mouse.y * gimbal_scale.angle_keyboard * 0.5f;
                    gimbal.yaw_angle.ref -= rc.mouse.x * gimbal_scale.angle_keyboard;
                }
                gimbal_pid_calc();
                break;
            }
            default:break;
        }
        gimbal_data_output();
        status.task.gimbal = 1;
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
