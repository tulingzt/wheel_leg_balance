#include "prot_power.h"
#include "mode_switch_task.h"
#include "can_comm.h"
#include "prot_judge.h"
#include "drv_dji_motor.h"
#include "string.h"

#define  TOQUE_COEFFICIENT      1.99688994e-6f
#define  K0 1.453e-07f
#define	 R0 1.23e-07f
#define  P0 3.081f

supercap_t supercap;
power_control_t power_control;

void power_init(void)
{
    memset(&supercap, 0, sizeof(supercap_t));
    memset(&power_control, 0, sizeof(power_control_t));
    power_control.judge_power_buffer = 60.0f;
    power_control.judge_max_power    = 55;
    power_control.min_buffer         = 30;
    power_control.limit_kp           = 0.4f;
    power_control.limit_power        = 170.0f;//170 250
    supercap.max_volage              = 23.6f;
    supercap.min_volage              = 10.0f;
    supercap.volume_percent          = 0;
    supercap.volage                  = supercap.min_volage;
}

void power_judge_update(void)
{
    power_control.judge_chassis_power = power_heat_data.chassis_power;
    power_control.judge_max_power     = robot_status.chassis_power_limit;
    power_control.judge_power_buffer  = power_heat_data.buffer_energy;
}

float motor_power_calcu(float current, float wheel_speed_fdb)
{
    float power;
    power = TOQUE_COEFFICIENT * current * wheel_speed_fdb +
                (double)K0 * wheel_speed_fdb * wheel_speed_fdb +
                (double)R0 * current * current 
								+ P0;
    return power;
}

void power_limit_current(void)
{
//    power_judge_update();
//    power_control.total_power_wheel = 0;
//    //未限功率前预测功率
//    for (int i = 0; i < 2; i++) {
//        power_control.give_power_wheel[i] =  motor_power_calcu(driver_motor[i].tx_current, driver_motor[i].speed_rpm);
//        if (power_control.give_power_wheel[i] < 0)
//            continue;
//        power_control.total_power_wheel += power_control.give_power_wheel[i];
//    }
//    //功率超限重分配 限电流方案
//    if (power_control.total_power_wheel >= power_control.limit_power) {
//        float a = 0, b = 0, c = 2 * P0 - power_control.limit_power;
//        for (int i = 0; i < 2; i++) {
//            a += R0 * driver_motor[i].tx_current * driver_motor[i].tx_current;
//            b += TOQUE_COEFFICIENT * driver_motor[i].speed_rpm * driver_motor[i].tx_current;
//            c += K0 * driver_motor[i].speed_rpm * driver_motor[i].speed_rpm;
//        }
//        if (b * b - 4 * a * c >= 0) {//有解
//            power_control.power_scale = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
//        } else {
//            power_control.power_scale = -b / 2 / a;
//        }
//    } else {
//        power_control.power_scale = 1.0f;
//    }
    power_judge_update();
    power_control.total_power_wheel = 0;
    //未限功率前预测功率
    for (int i = 0; i < 2; i++) {
        power_control.give_power_wheel[i] = motor_power_calcu(driver_motor[i].tx_current, driver_motor[i].speed_rpm);
        if (power_control.give_power_wheel[i] < 0)
            continue;
        power_control.total_power_wheel += power_control.give_power_wheel[i];
    }
    if (power_control.total_power_wheel >= power_control.limit_power) {//功率超限重分配
        float a = 0, b = 0, c = 2 * P0 - power_control.limit_power;
        for (int i = 0; i < 2; i++) {
            a += R0 * driver_motor[i].tx_current * driver_motor[i].tx_current;
            b += TOQUE_COEFFICIENT * driver_motor[i].speed_rpm * driver_motor[i].tx_current;
            c -= K0 * driver_motor[i].speed_rpm * driver_motor[i].speed_rpm;
        }
        if (b * b - 4 * a * c >= 0) {//有解
            power_control.power_scale = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
        } else {
            power_control.power_scale = -b / 2 / a;
        }
    } else {
        power_control.power_scale = 1.0f;
    }
    //限制电流
//    driver_motor[0].tx_current = power_control.power_scale * driver_motor[0].tx_current;
//    driver_motor[1].tx_current = power_control.power_scale * driver_motor[1].tx_current;
//    //限电流功率后预测功率
//    power_control.total_power_wheel = 0;
//    for (int i = 0; i < 2; i++) {
//        power_control.give_power_wheel[i] =  motor_power_calcu(driver_motor[i].tx_current, driver_motor[i].speed_rpm);
//        power_control.total_power_wheel += power_control.give_power_wheel[i];
//    }
//    if (power_control.judge_power_buffer < power_control.min_buffer) {//限制速度 后面再写
//        
//    }
}

float power_limit_speed(void)
{
    power_judge_update();
    power_control.total_power_wheel = 0;
    //未限功率前预测功率
    for (int i = 0; i < 2; i++) {
        power_control.give_power_wheel[i] =  motor_power_calcu(driver_motor[i].tx_current, driver_motor[i].speed_rpm);
        if (power_control.give_power_wheel[i] < 0)
            continue;
        power_control.total_power_wheel += power_control.give_power_wheel[i];
    }
    //功率超限重分配 限速度方案
    float a = 2 * K0;
    float b = 0;
    float c = 2 * P0 - power_control.limit_power;
    for (int i = 0; i < 2; i++) {
        if (driver_motor[i].tx_current > 0) {
            b += 2 * TOQUE_COEFFICIENT * driver_motor[i].tx_current;
        } else {
            b -= 2 * TOQUE_COEFFICIENT * driver_motor[i].tx_current;
        }
        c += R0 * driver_motor[i].tx_current * driver_motor[i].tx_current;
    }
    if (b * b - 4 * a * c >= 0) {
        power_control.power_scale = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
    } else {
        power_control.power_scale = -b / 2 / a;
    }
    return power_control.power_scale;
}

void power_get_data(uint8_t *data)
{
    //0x100
    float cap_voltage_buf;
    float chassis_current_buf;
    
    memcpy(&cap_voltage_buf, data, 4);
    memcpy(&chassis_current_buf, data + 4, 4);
    
    power_control.online = 1;
    supercap.volage = cap_voltage_buf;
    supercap.volume_percent = (supercap.volage - supercap.min_volage) / (supercap.max_volage - supercap.min_volage) * 100.0f;
    supercap.current = chassis_current_buf;
    //0x101
//    uint8_t cap_state_buf;
//    memcpy(&cap_state_buf,data,1);
//    supercap.state.cap_v_over = cap_state_buf  & 1;
//    supercap.state.cap_v_low = cap_state_buf >> 1 & 1;
//    supercap.state.bat_v_over = cap_state_buf >> 2 & 1;
//    supercap.state.bat_v_low = cap_state_buf >> 3 & 1;
//    supercap.state.cap_i_over = cap_state_buf >> 4 & 1;
//    supercap.state.chassis_i_over = cap_state_buf >> 5 & 1;
//    supercap.state.chassis_msg_miss = cap_state_buf >> 6 & 1;
//    supercap.state.judge_msg_miss = cap_state_buf >> 7;
//    memcpy(&supercap.POWER_MODE,data+1,sizeof(power_mode));

}

uint8_t power_check_offline(void)
{
    if (power_control.online == 0) {
        return 1;
    } else {
        power_control.online = 0;
        return 0;
    }
}

