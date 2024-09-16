#include "chassis_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "wlr.h"
#include "can_comm.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_imu.h"
#include "prot_dr16.h"
#include "prot_power.h"
#include "pid.h"
#include "math_lib.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "string.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "KNN.h"
#include "func_generator.h"

#define JOINT_MOTOR_RESET_TORQUE 2.0f
#define JOINT_MOTOR_RESET_ERROR 0.005f

ramp_t chassis_x_ramp;
ramp_t chassis_y_ramp;
kalman_filter_t kal_3508_vel[2];
kalman_filter_t kal_wy;
FGT_sin_t FGT_sin_chassis;
chassis_t chassis;
chassis_scale_t chassis_scale = {
    .remote = 1.0f/660*2.0f,
    .keyboard = 2.5f
};

static void chassis_ramp(void)
{
    if (rc.kb.bit.W) {
        ramp_calc(&chassis_x_ramp, chassis_scale.keyboard);
    } else if (rc.kb.bit.S) {
        ramp_calc(&chassis_x_ramp, -chassis_scale.keyboard);
    } else {
        ramp_calc(&chassis_x_ramp, 0);
    }
    if (rc.kb.bit.D) {
        ramp_calc(&chassis_y_ramp, chassis_scale.keyboard);
    } else if (rc.kb.bit.A) {
        ramp_calc(&chassis_y_ramp, -chassis_scale.keyboard);
    } else {
        ramp_calc(&chassis_y_ramp, 0);
    }
}

static void joint_motor_reset(void)
{
    static motor_reset_t motor_reset[4];
    uint32_t thread_wake_time = osKernelSysTick();
    //清除上次复位数据和电机数据 重新初始化
    dji_motor_set_torque(&driver_motor[0], 0);
    dji_motor_set_torque(&driver_motor[1], 0);
    chassis.joint_motor_reset = 0;
    for (int i = 0; i < 4; i++) {
        motor_reset[i].reset_flag = 0;
//        ht_motor_set_control_cmd(&joint_motor[i], CMD_MOTOR_MODE);
//        osDelayUntil(&thread_wake_time, 1);
        ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
//        osDelayUntil(&thread_wake_time, 1);
    }
    //未解锁 等待
    while (!lock_flag) {
        status.task.chassis = 1;
        osDelayUntil(&thread_wake_time, 2);
    }
    //开始复位
    while (!chassis.joint_motor_reset) {
        if (motor_reset[0].reset_flag == 1 && motor_reset[1].reset_flag == 1 && \
            motor_reset[2].reset_flag == 1 && motor_reset[3].reset_flag == 1)
            chassis.joint_motor_reset = 1;
        else
            chassis.joint_motor_reset = 0;
        for (int i = 0; i < 4; i++) {
            if (motor_reset[i].reset_flag == 0) {
                if (i == 0 || i == 2)
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, -JOINT_MOTOR_RESET_TORQUE);
                else
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, JOINT_MOTOR_RESET_TORQUE);
                if (fabs(motor_reset[i].last_position - joint_motor[i].position) < JOINT_MOTOR_RESET_ERROR)
                    motor_reset[i].stop_cnt++;
                else
                    motor_reset[i].stop_cnt = 0;
                if (motor_reset[i].stop_cnt >= 100) {
                    motor_reset[i].reset_flag = 1;
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
                    ht_motor_set_control_cmd(&joint_motor[i], CMD_ZERO_POSITION);
                } else {
                    motor_reset[i].last_position = joint_motor[i].position;
                }
            } else {
                ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
            }
        }
        status.task.chassis = 1;
        osDelayUntil(&thread_wake_time, 2);
    }
    reset_flag = 0;
}

static void chassis_init()
{
    memset(&chassis, 0, sizeof(chassis_t));
    memset(&chassis_x_ramp, 0, sizeof(ramp_t));
    memset(&chassis_y_ramp, 0, sizeof(ramp_t));
    wlr_init();
	joint_motor_reset();
    ramp_init(&chassis_x_ramp, 0.01f, -chassis_scale.keyboard, chassis_scale.keyboard);//0.02 0.1s达到最大
    ramp_init(&chassis_y_ramp, 0.01f, -chassis_scale.keyboard, chassis_scale.keyboard);
    wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
    wlr.yaw_offset = 1.7f;
    
    for(int i = 0; i < 2; i++) {
        kalman_filter_init(&kal_3508_vel[i], 1, 0, 1);
        kal_3508_vel[i].A_data[0] = 1;
        kal_3508_vel[i].H_data[0] = 1;
        kal_3508_vel[i].Q_data[0] = 1;
        kal_3508_vel[i].R_data[0] = 200;
    }
    
    kalman_filter_init(&kal_wy, 1, 0, 1);
    kal_wy.A_data[0] = 1;
    kal_wy.H_data[0] = 1;
    kal_wy.Q_data[0] = 1;
    kal_wy.R_data[0] = 100;
    	FGT_sin_init (&FGT_sin_chassis,2,2,500,1.0f,0);
    chassis.init = 1;
}

float spin_limit;
static void chassis_mode_switch(void)
{
    /* 系统历史状态 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    /* 按键扫描 */
    key_scan(KEY_CHASSIS_ROTATE);
    key_scan(KEY_CHASSIS_HEIGHT);
    key_scan(KEY_CHASSIS_HEIGHT2);
    key_scan(KEY_CHASSIS_FIGHT);
    key_scan(KEY_CHASSIS_PRONE);
    key_scan(KEY_CHASSIS_UNFOLLOW);
    key_scan(KEY_CHASSIS_POWER);
    /* 底盘状态切换 */
    spin_limit = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    switch (ctrl_mode) {
    case PROTECT_MODE: { //能量模式和保护模式下，底盘行为相同
        chassis.mode = CHASSIS_MODE_PROTECT;
        wlr.prone_flag = 0;
        break;
    }
    case REMOTER_MODE: {
        if (last_ctrl_mode != REMOTER_MODE || chassis.mode == CHASSIS_MODE_PROTECT) { //切入遥控模式，初始化底盘模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        }
        /* 底盘小陀螺模式 */
        static uint8_t spin_flag; //单次触发使能标志
        if (ABS(rc.ch3) < 10) { //使能底盘模式切换
            spin_flag = 1;
        }
        if (rc.ch3 == 660 && spin_flag == 1) {
            if ((chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 || chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2)
                 && 0 < spin_limit && spin_limit < 1.7f) {
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                spin_flag = 0;
            } else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW) {
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE1;
                spin_flag = 0;
            }
        }
        if (rc.ch3 == -660 && spin_flag == 1) {
            if ((chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 || chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2)
                 && -1.7f < spin_limit && spin_limit < 0) {
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                spin_flag = 0;
            } else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW) {
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE2;
                spin_flag = 0;
            }
        }

        /* 遥控器注释底盘 */
        if (rc_fsm_check(RC_LEFT_RU) || rc_fsm_check(RC_RIGHT_RU)) { //遥控器注释底盘
            chassis.mode = CHASSIS_MODE_PROTECT;
        }
        break;
    }
    case KEYBOARD_MODE: { //键盘模式下：(跟随，陀螺，迎敌三种模式相互切换),(跟随与补给模式相互切换)
        /* 底盘模式切换 */
        switch (chassis.mode) {
        case CHASSIS_MODE_KEYBOARD_FOLLOW: { //键盘跟随模式下
            if(kb_status[KEY_CHASSIS_PRONE]) { //趴倒模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_PRONE;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入键盘陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            } else if (kb_status[KEY_CHASSIS_UNFOLLOW]) { //进入不跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_ROTATE: { //键盘陀螺模式下
            if ((!kb_status[KEY_CHASSIS_ROTATE]) && -1.7f < spin_limit && spin_limit < 0) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: { //键盘迎敌模式下
            if (!kb_status[KEY_CHASSIS_FIGHT]) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_UNFOLLOW]) { //进入不跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW: { //键盘不跟随模式下
            if (!kb_status[KEY_CHASSIS_UNFOLLOW]) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
                kb_status[KEY_CHASSIS_FIGHT]=1;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            if(!kb_status[KEY_CHASSIS_PRONE]) { //趴倒模式
                wlr.prone_flag = 0;
                wlr.high_flag = 0;
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            }
            break;
        }
        default: {
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            break;
        }
        }
        break;
    }
    default: break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
    /* 状态标志复位 */
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_ROTATE)
        key_status_clear(KEY_CHASSIS_ROTATE);
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_FIGHT)
        key_status_clear(KEY_CHASSIS_FIGHT);
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_UNFOLLOW)
        key_status_clear(KEY_CHASSIS_UNFOLLOW);
    if(chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE)
        key_status_clear(KEY_CHASSIS_PRONE);
}

static void chassis_data_input(void)
{
    static chassis_mode_e last_chassis_mode = CHASSIS_MODE_PROTECT;
    //速度输入 模式输入
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {  //底盘保护模式
            chassis.input.vx = 0;
            chassis.input.vy = 0;
            wlr.ctrl_mode = 0;
            key_status_clear(KEY_CHASSIS_HEIGHT);
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:  //底盘遥控跟随模式 底盘遥控陀螺模式
        case CHASSIS_MODE_REMOTER_ROTATE1:
        case CHASSIS_MODE_REMOTER_ROTATE2: {
            chassis.input.vx = rc.ch4 * chassis_scale.remote; //目标速度
            chassis.input.vy = rc.ch3 * chassis_scale.remote;
			wlr.ctrl_mode = 2; //轮腿模式
           //高度模式
           if (wlr.ctrl_mode == 2) {  //轮腿模式下才可控制腿长
			   if (rc.sw2 == RC_UP) {
                   wlr.jump_flag = 0;
                   wlr.high_flag = 0;
                   wlr.power_flag = 1;
               } else if (rc.sw2 == RC_MI) {
				   wlr.jump_flag = 0;
                   wlr.high_flag = 1;
                   wlr.power_flag = 1;
                   wlr.jump_cnt = 0;
               } else if (rc.sw2 == RC_DN && wlr.jump_flag == 0 && rc_fsm_check(RC_LEFT_RD))  
                    wlr.high_flag = 2;
                 else if (rc.sw2 == RC_DN && wlr.jump_flag == 0)
                    wlr.jump_flag = 1;
                 else {
               wlr.high_flag = 0;
           }
       }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FOLLOW:  //跟随模式可以跳跃  底盘键盘跟随/陀螺/迎敌模式
            if (KEY_PRESS_JUMP && wlr.jump_flag == 0 && KEY_PRESS_POWER)
                wlr.jump_flag = 1;
            else if (!KEY_PRESS_JUMP)
                wlr.jump_flag = 0;
        case CHASSIS_MODE_KEYBOARD_ROTATE:
        case CHASSIS_MODE_KEYBOARD_FIGHT:
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            //速度选择
            if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT ||
                chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE ||
                chassis.mode == CHASSIS_MODE_KEYBOARD_UNFOLLOW ||
                wlr.high_flag == 2) {
                wlr.power_flag = 0;
                chassis_scale.keyboard = 1.0f;  //迎敌模式下
            } else if (KEY_PRESS_POWER) {
                wlr.power_flag = 1;
                chassis_scale.keyboard = 2.5f;  //高速模式下
            } else {
                wlr.power_flag = 1;
                chassis_scale.keyboard = 2.0f;  //普通模式下
            }
            //速度输入
            chassis_ramp();
            chassis.input.vx = chassis_x_ramp.out;
            chassis.input.vy = chassis_y_ramp.out;
            //控制模式
            wlr.ctrl_mode = 2; //键盘直接是轮腿模式
            if (chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
                wlr.prone_flag = 1;
            }
            //高度模式
            if (wlr.high_flag == 0) {
                if (kb_status[KEY_CHASSIS_HEIGHT] == KEY_RUN && 
                    chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 1;
                }
                if (kb_status[KEY_CHASSIS_HEIGHT2] == KEY_RUN && 
                    chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 2;
                }
            } else if (wlr.high_flag == 1) {
                if (kb_status[KEY_CHASSIS_HEIGHT2] == KEY_RUN && 
                    chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 2;
                    key_status_clear(KEY_CHASSIS_HEIGHT);
                } else if (kb_status[KEY_CHASSIS_HEIGHT] != KEY_RUN ||
                    chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 0;
                    key_status_clear(KEY_CHASSIS_HEIGHT);
                }
            } else if (wlr.high_flag == 2) {
                chassis_scale.keyboard = 1.0f;
                wlr.power_flag = 0;
                if (kb_status[KEY_CHASSIS_HEIGHT] == KEY_RUN && 
                    chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 1;
                    key_status_clear(KEY_CHASSIS_HEIGHT2);
                } else if (kb_status[KEY_CHASSIS_HEIGHT2] != KEY_RUN ||
                    chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
                    wlr.high_flag = 0;
                    key_status_clear(KEY_CHASSIS_HEIGHT2);
                }
            } else {
                wlr.high_flag = 0;
                key_status_clear(KEY_CHASSIS_HEIGHT);
                key_status_clear(KEY_CHASSIS_HEIGHT2);
            }
            break;
        }
        default:break;
    }
    //旋转数据输入
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {
            wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            if (gimbal.start_up)//完成起身，前方灯条
                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
            else//起身未完成，目标值等于反馈值
                wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2 * PI;                
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            //此yaw_err用于底盘前后都可跟随
//            wlr.yaw_err = circle_error(wlr.yaw_ref, wlr.yaw_fdb, 2 * PI);
//            if (wlr.yaw_err > PI / 2 || wlr.yaw_err < - PI / 2) {
//                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI - PI;
//            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: {
            wlr.yaw_ref = (float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }
        case CHASSIS_MODE_REMOTER_ROTATE1:
        case CHASSIS_MODE_REMOTER_ROTATE2:
        case CHASSIS_MODE_KEYBOARD_ROTATE: {
            if (last_chassis_mode != CHASSIS_MODE_REMOTER_ROTATE1 && last_chassis_mode != CHASSIS_MODE_REMOTER_ROTATE2 && last_chassis_mode != CHASSIS_MODE_KEYBOARD_ROTATE)
                wlr.yaw_ref = -chassis_imu.yaw;
            if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1) {
                wlr.yaw_ref += (float)(CHASSIS_ROTATE_SPEED +FGT_sin_calc(&FGT_sin_chassis)) * 0.002f;
                wlr.yaw_fdb = -chassis_imu.yaw;
                wlr.wz_ref = 0;
            } else {
                wlr.yaw_ref -= (float)(CHASSIS_ROTATE_SPEED +FGT_sin_calc(&FGT_sin_chassis))  * 0.002f;
                wlr.yaw_fdb = -chassis_imu.yaw;
                wlr.wz_ref = 0;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW: {
            if (last_chassis_mode != CHASSIS_MODE_KEYBOARD_UNFOLLOW)
                wlr.yaw_ref = -chassis_imu.yaw;
            wlr.yaw_fdb = -chassis_imu.yaw;
            wlr.wz_ref = 0;
            break;
        }
        default:break;
    }
    last_chassis_mode = chassis.mode;
    //速度坐标系换算 
    if (wlr.yaw_ref < 0)
        wlr.yaw_ref += 2 * PI;
    else if (wlr.yaw_ref > 2 * PI)
        wlr.yaw_ref -= 2 * PI;
    //此yaw_err用于平移速度体系换算
    wlr.yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    //小陀螺偏置
    if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 ||
        chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2 ||
        chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE) {
        wlr.yaw_err += wlr.yaw_offset;
    }
    chassis.output.vx = chassis.input.vx * arm_cos_f32(wlr.yaw_err) - chassis.input.vy * arm_sin_f32(wlr.yaw_err);
    chassis.output.vy = chassis.input.vx * arm_sin_f32(wlr.yaw_err) + chassis.input.vy * arm_cos_f32(wlr.yaw_err);
    //此yaw_err用于纠正yaw_set与yaw_fdb在360之间的跳变差
    wlr.yaw_err = circle_error(wlr.yaw_ref, wlr.yaw_fdb, 2 * PI);
    wlr.yaw_ref = wlr.yaw_fdb + wlr.yaw_err;
	wlr.v_ref = chassis.output.vx;
    wlr.s_ref += (wlr.v_ref * 0.001f * 2);
    //陀螺仪数据输入
    wlr.roll_fdb    = -chassis_imu.rol;
    wlr.pit_fdb     = -chassis_imu.pit;
    
    kal_wy.measured_vector[0] = -chassis_imu.wy;
    kalman_filter_update(&kal_wy);
    wlr.wy_fdb = kal_wy.filter_vector[0];
    
//    wlr.wy_fdb      = -chassis_imu.wy;//
    wlr.wz_fdb      = -chassis_imu.wz;
    wlr.az_fdb      =  chassis_imu.az;
    //电机数据输入
    wlr.side[0].q4 =  joint_motor[0].position - joint_motor[0].zero_point;
    wlr.side[0].q1 =  joint_motor[1].position + joint_motor[1].zero_point + PI;
    wlr.side[0].w4 =  joint_motor[0].velocity;
    wlr.side[0].w1 =  joint_motor[1].velocity;
    wlr.side[0].t4 =  joint_motor[0].torque;
    wlr.side[0].t1 =  joint_motor[1].torque;
    wlr.side[0].qy = -driver_motor[0].position;
    //wlr.side[0].wy = -driver_motor[0].velocity;//可能需要加滤波
    
    kal_3508_vel[0].measured_vector[0] = -driver_motor[0].velocity;
    kalman_filter_update(&kal_3508_vel[0]);
    wlr.side[0].wy = kal_3508_vel[0].filter_vector[0];

    wlr.side[1].q1 = -joint_motor[2].position + joint_motor[2].zero_point + PI;
    wlr.side[1].q4 = -joint_motor[3].position - joint_motor[3].zero_point;
    wlr.side[1].w1 = -joint_motor[2].velocity;
    wlr.side[1].w4 = -joint_motor[3].velocity;
    wlr.side[1].t1 = -joint_motor[2].torque;
    wlr.side[1].t4 = -joint_motor[3].torque;
    wlr.side[1].qy =  driver_motor[1].position;
    //wlr.side[1].wy =  driver_motor[1].velocity;//可能需要加滤波
    
    kal_3508_vel[1].measured_vector[0] = driver_motor[1].velocity;
    kalman_filter_update(&kal_3508_vel[1]);
    wlr.side[1].wy = kal_3508_vel[1].filter_vector[0];
    for (int i = 0; i < 10; i++)
    input_data[i] = lqr.X_fdb[i];
    
    input_data[10] =  wlr.side[0].Fn_kal;
    input_data[11] =  wlr.side[1].Fn_kal;
 
}

static void chassis_data_output(void)
{
    if (wlr.ctrl_mode == 0) {//保护模式
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    } else if (wlr.ctrl_mode == 2) {//力控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1], wlr.side[1].Tw);
        if (wlr.prone_flag) {
            ht_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0.1, 0);//0.03 0.5
            ht_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0.1, 0);
            ht_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0.1, 0);
            ht_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0.1, 0);
        } else {
            ht_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, wlr.side[0].T4);//0.03 0.5
            ht_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, wlr.side[0].T1);
            ht_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, -wlr.side[1].T1);
            ht_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, -wlr.side[1].T4);
        }
    } else if (wlr.ctrl_mode == 1) {//位控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1], wlr.side[1].Tw);
        ht_motor_set_control_para(&joint_motor[0],  wlr.side[0].P4 + joint_motor[0].zero_point,      0, 10, 2,  2);
        ht_motor_set_control_para(&joint_motor[1],  wlr.side[0].P1 - joint_motor[1].zero_point - PI, 0, 10, 2, -2);
        ht_motor_set_control_para(&joint_motor[2], -wlr.side[1].P1 + joint_motor[2].zero_point + PI, 0, 10, 2,  2);
        ht_motor_set_control_para(&joint_motor[3], -wlr.side[1].P4 - joint_motor[3].zero_point,      0, 10, 2, -2);
    } else {//错误时
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    }
}

void chassis_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    power_init();
    for(;;)
    {
        if (chassis.init == 0) {
            chassis_init();
        }
        thread_wake_time = osKernelSysTick();
//        taskENTER_CRITICAL();
        chassis_mode_switch();
        chassis_data_input();
        wlr_control();
        chassis_data_output();
        status.task.chassis = 1;
//        power_limit_current();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
