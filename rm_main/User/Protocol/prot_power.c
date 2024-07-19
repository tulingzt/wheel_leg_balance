//#include "prot_power.h"
//#include "mode_switch_task.h"
//#include "can_comm.h"
//#include "prot_judge.h"
//#include "drv_dji_motor.h"
//#include "string.h"

//#define  K0 0
//#define	 R0 0
//#define  P0 0

//supercap_t supercap;
//power_control_t power_control;

//void power_init(void)
//{
//    memset(&supercap, 0, sizeof(supercap_t));
//    memset(&power_control, 0, sizeof(power_control_t));
//    power_control.judge_power_buffer = 60.0f;
//    power_control.judge_max_power    = 55;
//    power_control.min_buffer         = 30;
//    power_control.limit_kp           = 0.4f;
//    supercap.max_volage              = 23.6f;
//    supercap.min_volage              = 10.0f;
//    supercap.volume_percent          = 0;
//    supercap.volage                  = supercap.min_volage;
//}

//void power_judge_update(void)
//{
//    power_control.judge_chassis_power = power_heat_data.chassis_power;
//    power_control.judge_max_power     = robot_status.chassis_power_limit;
//    power_control.judge_power_buffer  = power_heat_data.buffer_energy;
//}

//float motor_power_calcu(float current, float wheel_speed_fdb)
//{
//    float power;
//    power = K0 * current * wheel_speed_fdb +
//            R0 * current * current + P0;
//    return power;
//}

//void supercap_control(void)
//{
//    power_control.give_power_wheel[0] =  motor_power_calcu(driver_motor[0].tx_current, driver_motor[0].speed_rpm);
//    power_control.give_power_wheel[1] =  motor_power_calcu(driver_motor[1].tx_current, driver_motor[1].speed_rpm);
//    power_control.total_power_wheel = power_control.give_power_wheel[0] + power_control.give_power_wheel[1];
//    
//    if (power_control.judge_power_buffer < power_control.min_buffer) {//限制速度 后面再写
//        
//    }
//}

//void power_get_data(uint8_t *data)
//{
//    float cap_voltage_buf;
//    float chassis_current_buf;
//    
//    memcpy(&cap_voltage_buf, data, 4);
//    memcpy(&chassis_current_buf, data + 4, 4);
//    
//    supercap.volage = cap_voltage_buf;
//    supercap.volume_percent = (supercap.volage - supercap.min_volage) / (supercap.max_volage - supercap.min_volage) * 100.0f;
//    supercap.current = chassis_current_buf;
//}

#include "prot_power.h"
#include "mode_switch_task.h"
#include "can_comm.h"
#include "prot_judge.h"
#include "string.h"

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
    supercap.max_volage              = 23.6f;
    supercap.limit_volage            = 20.0f;
    supercap.min_volage              = 13.5f;
    supercap.volume_percent          = 0;
    supercap.volage                  = supercap.min_volage;
}

void power_judge_update(void)
{
    power_control.judge_chassis_power = power_heat_data.chassis_power;
    power_control.judge_max_power     = robot_status.chassis_power_limit;
    power_control.judge_power_buffer  = power_heat_data.buffer_energy;
}

void supercap_mode_update(void)
{
    if (!lock_flag) {
        supercap.mode = CAP_PROTECT_MODE;
    } else if (ctrl_mode == PROTECT_MODE) {
        if (supercap.volage > supercap.max_volage || supercap.volage < supercap.min_volage) {
            supercap.mode = CAP_PROTECT_MODE;
        } else {
            supercap.mode = CAP_CHARGE_MODE;
        }
    } else {
        if (supercap.volage >= supercap.min_volage) {
            supercap.mode = CAP_DISCHARGE_MODE;
        } else if (power_control.judge_power_buffer <= power_control.min_buffer) {
            supercap.mode = CAP_PROTECT_MODE;
        } else{
            supercap.mode = CAP_CHARGE_MODE;
        }
    }
}

void supercap_control(void)
{
    if (supercap.mode == CAP_PROTECT_MODE) {
        supercap.charge_power_ref = 0;
        supercap.charge_current_ref = 0;
    } else if (supercap.mode == CAP_CHARGE_MODE) {
        supercap.charge_power_fdb = power_control.source_power - power_control.chassis_power - 10.0f;
        if (supercap.charge_power_fdb < 0)
            supercap.charge_power_fdb = 0;
        supercap.charge_power_ref = power_control.judge_max_power - power_control.chassis_power - 15.0f;
        if (supercap.charge_power_ref < 0)
            supercap.charge_power_ref = 0;
        supercap.charge_current_ref = supercap.charge_power_ref / supercap.volage;
        if (supercap.charge_current_ref <= 0)
            supercap.charge_current_ref = 0;
        else if (supercap.charge_current_ref >= 10)
            supercap.charge_current_ref = 10;
    } else if (supercap.mode == CAP_DISCHARGE_MODE) {
        supercap.charge_power_ref = power_control.judge_max_power - 10.0f;
        if (supercap.charge_power_ref < 0)
            supercap.charge_power_ref = 0;
        supercap.charge_current_ref = supercap.charge_power_ref / supercap.volage;
        if (supercap.charge_current_ref <= 0)
            supercap.charge_current_ref = 0;
        else if (supercap.charge_current_ref >= 10)
            supercap.charge_current_ref = 10;
    }
    //限制电流，使充电电流不突变
    if (supercap.volage > supercap.limit_volage)
        supercap.charge_current_ref = supercap.charge_current_ref * (supercap.max_volage - supercap.volage) / (supercap.max_volage - supercap.limit_volage);
}

void power_output_data(void)
{
    uint8_t send_buff[8] = {0};
    send_buff[0] = supercap.mode;
    memcpy(send_buff + 1, &supercap.charge_current_ref, sizeof(supercap.charge_current_ref));
    can_std_transmit(CAN_CHANNEL_2, 0x021, send_buff);
}

void power_get_data(uint8_t *data)
{
    uint16_t supercap_voltage = 0;
    uint16_t source_power = 0;
    uint16_t chassis_power = 0;

    memcpy(&supercap_voltage, data, 2);
    memcpy(&source_power, data + 2, 2);
    memcpy(&chassis_power, data + 4, 2);
    //电容电压信息
    supercap.volage = supercap_voltage / 100.0f;
    supercap.volume_percent = (supercap.volage - supercap.min_volage) / (supercap.max_volage - supercap.min_volage) * 100.0f;
    //功率信息
    power_control.source_power = source_power / 100.0f;
    power_control.chassis_power = chassis_power / 100.0f;
}

