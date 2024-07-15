#include "rgb_task.h"
#include "control_def.h"

#include "mode_switch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"

#include "wlr.h"

#include "prot_dr16.h"
#include "prot_vision.h"
#include "prot_power.h"

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "drv_ws2812b.h"

void read_status(void)
{
    if (((rc.sw2 == RC_MI || rc.sw2 == RC_DN) && ctrl_mode == PROTECT_MODE) ||
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
    
    if (kb_status[KB_CTRL] == KEY_RUN) {
        rgb_change(3,2);
    } else if (kb_status[KEY_CHASSIS_LOWSPEED] == KEY_RUN) {
        rgb_change(3,3);
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

void rgb_task(void const *argu)
{
    for(;;)
    {
		read_status();
		rgb_output_data();
        osDelay(100);
    }
}
