#include "prot_power.h"
#include "mode_switch_task.h"
#include "can_comm.h"
#include "prot_judge.h"
#include "drv_dji_motor.h"
#include "string.h"

#define  K0 0
#define	 R0 0
#define  P0 0

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
    power = K0 * current * wheel_speed_fdb +
            R0 * current * current + P0;
    return power;
}

void supercap_control(void)
{
    power_control.give_power_wheel[0] =  motor_power_calcu(driver_motor[0].tx_current, driver_motor[0].speed_rpm);
    power_control.give_power_wheel[1] =  motor_power_calcu(driver_motor[1].tx_current, driver_motor[1].speed_rpm);
    power_control.total_power_wheel = power_control.give_power_wheel[0] + power_control.give_power_wheel[1];
    
    if (power_control.judge_power_buffer < power_control.min_buffer) {//限制速度 后面再写
        
    }
}

void power_get_data(uint8_t *data)
{
    float cap_voltage_buf;
    float chassis_current_buf;
    
    memcpy(&cap_voltage_buf, data, 4);
    memcpy(&chassis_current_buf, data + 4, 4);
    
    power_control.online = 1;
    supercap.volage = cap_voltage_buf;
    supercap.volume_percent = (supercap.volage - supercap.min_volage) / (supercap.max_volage - supercap.min_volage) * 100.0f;
    supercap.current = chassis_current_buf;
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

