#include "shoot_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "prot_judge.h"
#include "prot_dr16.h"
#include "prot_vision.h"
#include "data_buffer.h"
#include "cmsis_os.h"

#define SHOOT_SPEED_NUM 15
#define ABS(x) ((x>0)? (x): (-(x)))
#define TRIGGER_MOTOR_ECD_SINGLE   (32818.0f)  //拨盘一颗子弹转过的编码值 8191 * 36 / 8 = 36859.5f
#define TRIGGER_MOTOR_ECD_SERIES   (32818.0f)  //拨盘一颗子弹转过的编码值 8191 * 36 / 8 = 36859.5f

float MIN_HEAT = 40;        //热量控制裕量

static uint16_t frequency_cnt = 0;	//射击周期计算
static uint8_t  shoot_enable  = 1;  //单发使能标志
static float trigger_ecd_error;

shoot_t shoot;
//static buffer_t *shoot_speed_buffer;

static uint8_t single_shoot_reset(void)
{
    return (
        (rc.mouse.l == 0 && ctrl_mode == KEYBOARD_MODE)
        || (ABS(rc.ch5) < 10 && ctrl_mode == REMOTER_MODE)
    );
}

static uint8_t single_shoot_enable(void)
{
    return (
        shoot_enable
        && shoot.barrel.heat_remain >= MIN_HEAT
        && ((rc.mouse.l && ctrl_mode == KEYBOARD_MODE) || (rc.ch5 > 500 && ctrl_mode == REMOTER_MODE))
        && ABS(trigger_ecd_error) < TRIGGER_MOTOR_ECD_SINGLE
    );
}

static uint8_t series_shoot_enable(void)
{
    return (
        (      (ctrl_mode == REMOTER_MODE && vision.shoot_enable && (rc_fsm_check(RC_LEFT_RU) || rc_fsm_check(RC_RIGHT_RU))) //开启视觉连发
            || (ctrl_mode == REMOTER_MODE && rc.sw2 == RC_DN && !(rc_fsm_check(RC_LEFT_RU) || rc_fsm_check(RC_RIGHT_RU))) //开启遥控连发
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l && rc.mouse.r && vision.shoot_enable)
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l && rc.mouse.r == 0)
        )
        && shoot.barrel.heat_remain >= MIN_HEAT  //热量控制
        && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period  //射频控制
        && ABS(trigger_ecd_error) < TRIGGER_MOTOR_ECD_SERIES  //拨盘误差控制
    );
}

static void shoot_control(void)
{
    switch (shoot.trigger_mode) {
        case TRIGGER_MODE_PROTECT: { //拨盘保护模式，保持惯性，无力
            frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.barrel.shoot_period = 0;
            shoot.trigger_ecd.ref = trigger_motor.total_ecd;
            shoot.trigger_spd.pid.i_out = 0;
            shoot.trigger_output = 0;
            break;
        }
        case TRIGGER_MODE_STOP: { //拨盘停止模式，保持静止，有力
            frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.barrel.shoot_period = 0;
            
            shoot.trigger_ecd.ref = trigger_motor.total_ecd;
            shoot.trigger_spd.pid.i_out = 0;
            break;
        }
        case TRIGGER_MODE_SINGLE: { //拨盘单发模式，连续开枪请求，只响应一次
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (single_shoot_reset()) {
                shoot_enable = 1;
            }
            if (single_shoot_enable()) { //热量控制
                shoot_enable = 0;
                shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD_SINGLE;
                shoot.barrel.heat += 10;
            }
            break;
        }
        case TRIGGER_MODE_SERIES: { //拨盘连发模式，连续开枪请求，连续响应
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (series_shoot_enable()) { //一个周期打一颗
                frequency_cnt = 0;
                shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD_SERIES;//拨一颗子弹
                shoot.barrel.heat += 10;
            }
            break;
        }
        default:break;
    }
    switch (shoot.fric_mode) {
        case FRIC_MODE_PROTECT:
        case FRIC_MODE_STOP: {
            shoot.fric_spd[0].ref = 0;
            shoot.fric_spd[1].ref = 0;
            break;
        }
        case FRIC_MODE_RUN: {
            shoot.fric_spd[0].ref = -shoot.fric_speed_set;
            shoot.fric_spd[1].ref = shoot.fric_speed_set;
            break;
        }
        default:break;
    }
}

static void shoot_init(void)
{
    memset(&shoot, 0, sizeof(shoot_t));
    //发射器底层初始化
    pid_init(&shoot.fric_spd[0].pid, NONE, 0.0005f, 0, 0, 0, 0.8);
    pid_init(&shoot.fric_spd[1].pid, NONE, 0.0005f, 0, 0, 0, 0.8);
    pid_init(&shoot.trigger_ecd.pid, NONE, 0.3f, 0, 0.3f, 0, 5000);
    pid_init(&shoot.trigger_spd.pid, NONE, 0.0015f, 0, 0, 0, 1);
    //发射器模式初始化
    shoot.trigger_mode  = TRIGGER_MODE_PROTECT;
    shoot.fric_mode     = FRIC_MODE_PROTECT;
    //枪管参数初始化
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.fric_speed_set = 700;
    shoot.barrel.cooling_rate   = 10;
    shoot.barrel.heat_max       = 50;
    //历史射速反馈缓存区
//    shoot_speed_buffer = buffer_create(SHOOT_SPEED_NUM, sizeof(float));
}

static void shoot_pid_calc(void)
{
    for (int i = 0; i < 2; i++) {
        shoot.fric_spd[i].fdb = fric_motor[i].velocity;
        shoot.fric_output[i] = pid_calc(&shoot.fric_spd[i].pid, shoot.fric_spd[i].ref, shoot.fric_spd[i].fdb);
    }
    shoot.trigger_ecd.fdb = trigger_motor.total_ecd;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);
    shoot.trigger_spd.fdb = trigger_motor.speed_rpm;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);
}

static void shoot_data_output(void)
{
    if (shoot.fric_mode == FRIC_MODE_PROTECT) {
        dji_motor_set_torque(&fric_motor[0], 0);
        dji_motor_set_torque(&fric_motor[1], 0);
    } else {
        dji_motor_set_torque(&fric_motor[0], shoot.fric_output[0]);
        dji_motor_set_torque(&fric_motor[1], shoot.fric_output[1]);
    }
    if (shoot.trigger_mode == TRIGGER_MODE_PROTECT) {
        dji_motor_set_torque(&trigger_motor, 0);
    } else {
        dji_motor_set_torque(&trigger_motor, shoot.trigger_output);
    }
}

static void shoot_param_update(void)
{
    //更新裁判系统数据
    if (robot_status.shooter_barrel_heat_limit != 0) {
        shoot.barrel.heat_max = robot_status.shooter_barrel_heat_limit;//枪管热量上限
        shoot.barrel.cooling_rate = robot_status.shooter_barrel_cooling_value;//枪管冷却速率
    }
    //更新模拟裁判系统数据
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //当前枪管(理论)热量
    if (shoot.barrel.heat < 0) shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //当前枪管(理论)剩余热量
}

static void shoot_test(void) {
//    static float last_shoot_speed;
//    shoot.barrel.bullet_spd.fdb = shoot_data.initial_speed;
//    if (ABS(shoot.barrel.bullet_spd.fdb - last_shoot_speed) > 1e-5f) {
//        ++shoot.barrel.shoot_cnt;//统计发射子弹数
//        buffer_put_noprotect(shoot_speed_buffer, &shoot.barrel.bullet_spd.fdb);
//        float vision_data_array[SHOOT_SPEED_NUM] = {0};
//        uint32_t real_num = ubf_pop_into_array_new2old(shoot_speed_buffer, vision_data_array, 0, SHOOT_SPEED_NUM);
//        arm_var_f32(vision_data_array, real_num, &shoot.barrel.bullet_spd.std);  //计算数据方差
//        shoot.barrel.bullet_spd.std = sqrt(shoot.barrel.bullet_spd.std);
//        last_shoot_speed = shoot.barrel.bullet_spd.fdb;
//    }
}

static void shoot_mode_switch(void)
{
    /* 更新裁判系统参数 */
    shoot_param_update();
    /* 模式切换 */
    switch (ctrl_mode) {
        case PROTECT_MODE: {
            shoot.trigger_period = TRIGGER_PERIOD;
            shoot.fric_mode = FRIC_MODE_STOP;
            shoot.trigger_mode = TRIGGER_MODE_PROTECT;
            break;
        }
        case REMOTER_MODE: {
            shoot.trigger_period = TRIGGER_PERIOD;
            /* 摩擦轮和拨盘模式切换 */
            switch (rc.sw2) {
                case RC_UP: {
                    shoot.fric_mode = FRIC_MODE_STOP;
                    shoot.trigger_mode = TRIGGER_MODE_STOP;
                    break;
                }
                case RC_MI: {
                    shoot.fric_mode = FRIC_MODE_RUN;
                    shoot.trigger_mode = TRIGGER_MODE_SINGLE;
                    break;
                }
                case RC_DN: {
                    shoot.fric_mode = FRIC_MODE_RUN;
                    shoot.trigger_mode = TRIGGER_MODE_SERIES;
                    break;
                }
                default: break;
            }
            if (rc_fsm_check(RC_LEFT_RD) || rc_fsm_check(RC_RIGHT_RD)) { //遥控器注释发射
                shoot.fric_mode = FRIC_MODE_STOP;
                shoot.trigger_mode = TRIGGER_MODE_PROTECT;
            }
            break;
        }
        case KEYBOARD_MODE: {
            /* 射频切换 */
            if (rc.mouse.r)
                shoot.trigger_period = TRIGGER_PERIOD2;
            else
                shoot.trigger_period = TRIGGER_PERIOD;
            /* 摩擦轮模式切换 */
            if (robot_status.power_management_shooter_output) {  //发射机构得到供电 
                shoot.fric_mode = FRIC_MODE_RUN;  //开关摩擦轮         
            } else {
                shoot.fric_mode = FRIC_MODE_PROTECT;  //摩擦轮断电，软件保护，禁用摩擦轮
            }
            /* 视觉模式切换 */
            if (rc.kb.bit.SHIFT) {
                vision.tx.data.aiming_mode = 2;
            } else if (rc.kb.bit.B) {
                vision.tx.data.aiming_mode = 1;
            } else {
                vision.tx.data.aiming_mode = 0;
            }
            /* 拨盘模式切换 */
            if (shoot.fric_mode != FRIC_MODE_RUN) {
                shoot.trigger_mode = TRIGGER_MODE_STOP;
            } else if (vision.tx.data.aiming_mode != 0) {
                shoot.trigger_mode = TRIGGER_MODE_SINGLE;
            } else {
                shoot.trigger_mode = TRIGGER_MODE_SERIES;
            }
            break;
        }
        default: break;
    }
}

void shoot_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    shoot_init();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
//        taskENTER_CRITICAL();
        shoot_mode_switch();    /* 发射器模式切换 */
        shoot_control();
        shoot_pid_calc();
        shoot_data_output();
        shoot_test();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
