#include "debug_task.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "stdint.h"
#include "prot_vision.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "wlr.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_imu.h"
#include "kalman_filter.h"
#include "func_generator.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "us_time.h"

us_time_t test_time;
kalman_filter_t test;
uint8_t debug_wave = 14;

void log_scope_data_pkg(void)
{
    switch(debug_wave) {
        case 1: {//云台pid调试
//            log_scope_get_data(gimbal.yaw_spd.ref);
//            log_scope_get_data(gimbal.yaw_spd.fdb);
//            log_scope_get_data(gimbal.yaw_angle.ref);
//            log_scope_get_data(gimbal.yaw_angle.fdb);
//            log_scope_get_data(gimbal.test_count);
//            log_scope_get_data(yaw_motor.tx_current);
            
            
            log_scope_get_data(-gimbal_imu.wy);
            log_scope_get_data(gimbal.pit_spd.fdb);
//            log_scope_get_data(gimbal.pit_spd.ref);
//            log_scope_get_data(gimbal.pit_spd.fdb);
//            log_scope_get_data(gimbal.pit_angle.ref);
//            log_scope_get_data(gimbal.pit_angle.fdb);
//            log_scope_get_data(gimbal.pit_output);
//            log_scope_get_data(pit_motor.tx_current);
            break;
        } case 2: {//拨盘pid调试
//            log_scope_get_data(shoot.trigger_spd.ref);
//            log_scope_get_data(shoot.trigger_spd.fdb);
//            log_scope_get_data(shoot.trigger_ecd.ref);
//            log_scope_get_data(shoot.trigger_ecd.fdb);
//            log_scope_get_data(shoot.trigger_output);
//            log_scope_get_data(trigger_motor.tx_current);
            break;
        } case 3: {//底盘yaw roll调试
            log_scope_get_data(wlr.wz_ref);
            log_scope_get_data(wlr.wz_fdb);
            log_scope_get_data(wlr.yaw_fdb + wlr.yaw_err);
            log_scope_get_data(wlr.yaw_fdb);
            
//            log_scope_get_data(wlr.roll_set);
//            log_scope_get_data(wlr.roll_fdb);
            break;
        } case 4: {//底盘功率调试
            log_scope_get_data(power_heat_data.chassis_power);
            log_scope_get_data(power_heat_data.buffer_energy);
            log_scope_get_data(supercap.volage);
            break;
        } case 5: {//支持力调试
            log_scope_get_data(wlr.side[0].Fn_fdb);
            log_scope_get_data(wlr.side[0].Fn_kal);
            log_scope_get_data(wlr.side[0].fly_cnt);
            log_scope_get_data(wlr.side[1].Fn_fdb);
            log_scope_get_data(wlr.side[1].Fn_kal);
            log_scope_get_data(wlr.side[1].fly_cnt);
            break;
        } case 6: {//摩擦轮速度
            log_scope_get_data(vmc[0].L_fdb);
            log_scope_get_data(vmc[1].L_fdb);
            log_scope_get_data(wlr.jump_flag);
            log_scope_get_data(rc.sw2);
            break;
        } case 7: {
              log_scope_get_data(wlr.side[0].wy * 0.065f);
              log_scope_get_data(wlr.side[1].wy * 0.065f);
              log_scope_get_data(wlr.side[0].fly_flag);
              log_scope_get_data(wlr.side[1].fly_flag);
              log_scope_get_data(wlr.side[0].Tw);
              log_scope_get_data(wlr.side[1].Tw);
              log_scope_get_data(-lqr.U_ref[0]);
              log_scope_get_data(-lqr.U_ref[1]);
            break;
        } case 8: {//腿部力
            log_scope_get_data(wlr.side[0].Fy);
            log_scope_get_data(wlr.side[0].T0);
            log_scope_get_data(wlr.side[1].Fy);
            log_scope_get_data(wlr.side[1].T0);
            
            log_scope_get_data(wlr.side[0].Fy);
            log_scope_get_data(wlr.side[0].T0);
            log_scope_get_data(wlr.side[1].Fy);
            log_scope_get_data(wlr.side[1].T0);
            break;
        } case 9: {//底盘状态观测
            log_scope_get_data(wlr.s_ref);
            log_scope_get_data(wlr.s_fdb);
            log_scope_get_data(wlr.v_ref);
            log_scope_get_data(wlr.v_fdb);
            log_scope_get_data(wlr.yaw_ref);
            log_scope_get_data(wlr.yaw_fdb);
            log_scope_get_data(wlr.wz_ref);
            log_scope_get_data(wlr.wz_fdb);
            log_scope_get_data(wlr.pit_fdb);
            log_scope_get_data(wlr.wy_fdb);
            break;
        } case 10: {//状态预测
            log_scope_get_data(wlr.side[0].predict_wy);
            log_scope_get_data(-wlr.side[0].wy);
            log_scope_get_data(wlr.side[1].predict_wy);
            log_scope_get_data(-wlr.side[1].wy);
            break;
        } case 11: {
//            log_scope_get_data(driver_motor[0].tx_current);
//            log_scope_get_data(driver_motor[0].rx_current);
//            log_scope_get_data(driver_motor[1].tx_current);
//            log_scope_get_data(driver_motor[1].rx_current);
            
            log_scope_get_data(driver_motor[0].position);
            log_scope_get_data(driver_motor[1].position);
            log_scope_get_data(joint_motor[0].position);
            log_scope_get_data(joint_motor[1].position);
            log_scope_get_data(joint_motor[2].position);
            log_scope_get_data(joint_motor[3].position);
            break;
        } case 12: {//底盘功率模型
            log_scope_get_data(power_heat_data.chassis_power);
//            log_scope_get_data(power_control.total_power);
//            log_scope_get_data(power_control.scaled_power);
            log_scope_get_data(power_heat_data.chassis_voltage);
            log_scope_get_data(power_heat_data.buffer_energy);
            log_scope_get_data(supercap.volage);
            log_scope_get_data(power_control.power_scale);
            break;
        } case 13: {//超电调试
//            log_scope_get_data(supercap.state.cap_v_over);
//            log_scope_get_data(supercap.state.cap_v_low);
//            log_scope_get_data(supercap.state.bat_v_over);
//            log_scope_get_data(supercap.state.bat_v_low );
//            log_scope_get_data(supercap.state.cap_i_over);
//            log_scope_get_data(supercap.state.chassis_i_over);
//            log_scope_get_data(supercap.state.chassis_msg_miss);
//            log_scope_get_data(supercap.state.judge_msg_miss);
            
            log_scope_get_data(supercap.volage);
            log_scope_get_data(supercap.current);
            break;
            } case 14: {//海泰电机力矩
//                log_scope_get_data(joint_motor[0].t);
//                log_scope_get_data(joint_motor[0].torque);
//                log_scope_get_data(joint_motor[1].t);
//                log_scope_get_data(joint_motor[1].torque);
//                log_scope_get_data(joint_motor[2].t);
//                log_scope_get_data(joint_motor[2].torque);
//                log_scope_get_data(joint_motor[3].t);
//                log_scope_get_data(joint_motor[3].torque);
                
                log_scope_get_data(driver_motor[0].t);
                log_scope_get_data(driver_motor[0].torque);
                log_scope_get_data(driver_motor[1].t);
                log_scope_get_data(driver_motor[1].torque);
                
//                log_scope_get_data(chassis_imu.rol);
//                log_scope_get_data(chassis_imu.pit);
//                log_scope_get_data(chassis_imu.yaw);
//                log_scope_get_data(chassis_imu.wy);
//                log_scope_get_data(-wlr.wy_fdb);
//                log_scope_get_data(chassis_imu.wz);
                break;
            } case 15: {
//                log_scope_get_data(-driver_motor[0].velocity);
//                log_scope_get_data(wlr.side[0].wy);
//                log_scope_get_data(-driver_motor[1].velocity);
//                log_scope_get_data(wlr.side[1].wy);
                
//                log_scope_get_data(vmc[0].L_ref);
//                log_scope_get_data(vmc[0].L_fdb);
//                
//                log_scope_get_data(pid_leg_length[0].p_out);
//                log_scope_get_data(pid_leg_length[0].i_out);
//                log_scope_get_data(pid_leg_length[0].d_out);
//                log_scope_get_data(pid_leg_length[0].output);
                
                // 查看六个状态变量的跟随情况
//                log_scope_get_data(lqr.X_ref[0]);
//                log_scope_get_data(lqr.X_fdb[0]);
//                
//                log_scope_get_data(lqr.X_ref[1]);
//                log_scope_get_data(lqr.X_fdb[1]);
//                
//                log_scope_get_data(lqr.X_fdb[2]);
//                log_scope_get_data(lqr.X_fdb[3]);
//                log_scope_get_data(lqr.X_fdb[4]);
//                log_scope_get_data(lqr.X_fdb[5]);
                
                // IMU
//                log_scope_get_data(chassis_imu.wy);
//                log_scope_get_data(chassis_imu.wz);
//                log_scope_get_data(gimbal_imu.wy);
//                log_scope_get_data(gimbal_imu.wz);
                
                // ROLL
//                log_scope_get_data(pid_roll.ref);
//                log_scope_get_data(pid_roll.fdb * 180/PI);
                //离地检测(wlr.side[i].fly_cnt
                log_scope_get_data(wlr.side[0].Fn_kal);
                log_scope_get_data(wlr.side[1].Fn_kal);
                
                log_scope_get_data(wlr.side[0].fly_cnt);
                log_scope_get_data(wlr.side[1].fly_cnt);
                
                log_scope_get_data(wlr.side[0].fly_flag && wlr.side[1].fly_flag);
                log_scope_get_data(wlr.side[0].fly_flag);
                log_scope_get_data(wlr.side[1].fly_flag);
                
                
                break;
            }
        default:break;
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        log_scope_data_output();
        osDelayUntil(&thread_wake_time, 5);
    }
}
