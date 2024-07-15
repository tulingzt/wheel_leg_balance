#include "mode_switch_task.h"
#include "prot_dr16.h"
#include "cmsis_os.h"

uint8_t lock_flag = 0;
uint8_t reset_flag = 0;
ctrl_mode_e ctrl_mode;

static void unlock_init(void) {
    if (rc.sw1 == RC_UP && rc.sw2 == RC_UP ) { //左拨杆置上，右拨杆置上
        if (rc.ch4 < -600 && rc.ch3 > 600) {
            lock_flag = 1;  //左控制杆拨至右下
        }
    }
}

static void sw1_mode_handler(void) { //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
    switch (rc.sw1) {
        case RC_UP: {
            ctrl_mode = PROTECT_MODE;break;
        }
        case RC_MI: {
            ctrl_mode = REMOTER_MODE;break;
        }
        case RC_DN: {
//        if (rc.mouse.r == 1) {
//            ctrl_mode = VISION_MODE;    //视觉模式，右键开启
//        } else {
            ctrl_mode = KEYBOARD_MODE;break;
        }
        default:break;
    }
}

static void remote_reset(void)
{
    //保护模式下右拨杆拨至左下
    if (rc.sw1 == RC_UP && rc.sw2 == RC_UP && rc.ch1 == -660 && rc.ch2 == -660) {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

void mode_switch_task(void const *argu)
{
    ctrl_mode = PROTECT_MODE;
    lock_flag = 0;
    for (;;) {
        if (!lock_flag) {
            unlock_init();  //解锁操作
        }
        else {
            remote_reset();
            sw1_mode_handler();  //根据左拨杆切换系统模式
        }
        osDelay(10);
    }
}
