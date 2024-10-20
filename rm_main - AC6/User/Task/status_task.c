#include "status_task.h"
#include "control_def.h"
#include "cmsis_os.h"

#include "mode_switch_task.h"
#include "chassis_task.h"
#include "wlr.h"

#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_vision.h"
#include "drv_ws2812b.h"
#include "iwdg.h"

status_t status;

void normal_status(void)
{
    if (((rc_fsm_check(RC_LEFT_LD) || rc_fsm_check(RC_RIGHT_LD)) && ctrl_mode == REMOTER_MODE) ||
        (rc.mouse.r == 1 && ctrl_mode == KEYBOARD_MODE)) {
        rgb_change(1,2);
    } else if (ctrl_mode == PROTECT_MODE) {
        rgb_change(1,0);
    } else {
        rgb_change(1,7);
    }

	if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW || \
        chassis.mode == CHASSIS_MODE_KEYBOARD_FOLLOW) {
        rgb_change(2,7);
    } else if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT) {
        rgb_change(2,2);
    } else if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 || \
                chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2 || \
               chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE) {
        rgb_change(2,1);
    } else if (chassis.mode == CHASSIS_MODE_KEYBOARD_UNFOLLOW) {
        rgb_change(2,3);
    } else {
        rgb_change(2,0);
    }
    
    if (KEY_PRESS_POWER && kb_status[KEY_CHASSIS_POWER] == KEY_END) {
        rgb_change(3,1);
    } else if (KEY_PRESS_POWER && kb_status[KEY_CHASSIS_POWER] == KEY_RUN) {
        rgb_change(3,2);
    } else {
        rgb_change(3,0);
    }
    
    if (wlr.high_flag == 2) {
        rgb_change(4,1);
    } else if (wlr.high_flag == 1) {
        rgb_change(4,7);
    }else if (wlr.prone_flag) {
        rgb_change(4,2);
    } else {
        rgb_change(4,0);
    }

    if (vision.tx.data.aiming_mode == 0) {
        rgb_change(5,2);
    } else if (vision.tx.data.aiming_mode == 1) {
        rgb_change(5,1);
    } else {
        rgb_change(5,3);
    }
    
    rgb_change(6,8);
    rgb_set_bright(6, supercap.volume_percent/20);
}

int iwdg_test = 1;
void status_task(void const* argument)
{
    for(;;)
    {
        if (status.task.comm == 1 &&
            status.task.gimbal == 1 &&
            status.task.chassis == 1 &&
            status.task.shoot == 1 &&
            status.task.mode_switch == 1) {
            status.task.comm = 0;
            status.task.gimbal = 0;
            status.task.chassis = 0;
            status.task.shoot = 0;
            status.task.mode_switch = 0;
//            HAL_IWDG_Refresh(&hiwdg1);
        }

        rc_fsm_init(rc.online);
        status.remote = rc_check_offline();
        status.vision = vision_check_offline();
        status.judge = judge_check_offline();
        status.power = power_check_offline();
        status.imu = imu_check_offline();
        status.dji_motor = dji_motor_check_offline();
        status.ht_motor = ht_motor_check_offline();
        
        if (status.remote == 0 && status.vision == 0 && status.judge == 0 && \
            status.power == 0 && status.imu == 0 && status.dji_motor == 0 && \
            status.ht_motor == 0) {
            status.all = 0;
        } else {
            status.all = 1;
        }
        
        if (status.ht_motor == 1) {
            chassis.init = 0;
        }
        
        if (rc_fsm_check(RC_LEFT_LU) || status.remote || ctrl_mode == PROTECT_MODE) { //遥控器切换DEBUG灯板
            rgb_change(1, status.remote);
            rgb_change(2, status.vision);
            rgb_change(3, status.judge);
            rgb_change(4, status.power);
            rgb_change(5, status.imu);
            rgb_change(6, status.all);
        } else if (rc_fsm_check(RC_RIGHT_RU)) { //遥控器切换DEBUG灯板
            rgb_change(1, status.dji_motor);
            rgb_change(2, status.ht_motor);
            rgb_change(3, 0);
            rgb_change(4, 0);
            rgb_change(5, 0);
            rgb_change(6, 0);
        } else {
            normal_status();
        }
		rgb_output_data();
        
        osDelay(100);
    }
}
