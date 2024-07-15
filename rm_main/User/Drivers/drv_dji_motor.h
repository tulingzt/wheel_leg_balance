#ifndef __DRV_DJI_MOTOR_H
#define __DRV_DJI_MOTOR_H

#include "can_comm.h"
#include "data_list.h"

/*-----------------------------------------
电机控制数据
型号以及对应电调    发送控制数据范围    对应物理量   转矩常数    力矩范围
2006(C610)          -10000~10000        +-10A       0.18N*m/A   +-1.8N*m
3508(C620)          -16384~16384        +-20A       0.3N*m/A    +-6N*m
6020                -30000~30000        ?           0.741N*m/A  ?
-----------------------------------------*/

//电机类型定义
#define DJI_2006_MOTOR 0x00
#define DJI_3508_MOTOR 0x01
#define DJI_6020_MOTOR 0x02

//发送数据范围
#define DJI_2006_MOTOR_DATA_RANGE 10000
#define DJI_3508_MOTOR_DATA_RANGE 16384
#define DJI_6020_MOTOR_DATA_RANGE 29000

//发送数据所代表的物理量范围
#define DJI_2006_MOTOR_CURRENT_RANGE 10
#define DJI_3508_MOTOR_CURRENT_RANGE 20
#define DJI_6020_MOTOR_CURRENT_RANGE 3.0f

//力矩常数
#define DJI_2006_MOTOR_TORQUE_CONSTANT 0.18f
#define DJI_3508_MOTOR_TORQUE_CONSTANT 0.3f
#define DJI_6020_MOTOR_TORQUE_CONSTANT 0.741f

//电机默认减速比
#define DJI_2006_ORIGINAL_REDUCTION_RATIO 36
#define DJI_3508_ORIGINAL_REDUCTION_RATIO 19.2f
#define DJI_6020_ORIGINAL_REDUCTION_RATIO 1

typedef struct
{
    uint32_t id;
    uint8_t data[8];
} can_std_msg_t;

typedef struct
{
    list_t list;
    //电机参数
    uint8_t motor_type;
    can_channel_e can_channel;
    uint32_t can_id;
    float reduction_ratio;              //当前减速比
    uint32_t send_cnt, receive_cnt;
    float err_percent;
    //控制数据
    float t;
    //反馈数据
    float position, velocity, torque;   //rad rad/s rad/s/s N*m
    //中间数据
    uint16_t ecd;                       //当前编码值         单位：ecd 0~8191
    int16_t  speed_rpm;                 //转速               单位：rpm
    int16_t  rx_current, tx_current;    //实际转矩电流值     单位：A
    uint8_t	 temperature;               //电机温度           单位：摄氏度
    uint16_t offset_ecd;                //刚上电时的编码值
    uint16_t last_ecd;                  //上次接收的编码值
    int32_t  total_ecd;                 //上电到现在转动的总编码值
    int32_t	 round_cnt;                 //已转圈数
} dji_motor_t;

extern dji_motor_t fric_motor[2];
extern dji_motor_t driver_motor[2];
extern dji_motor_t pit_motor, yaw_motor;
extern dji_motor_t trigger_motor;

void dji_motor_init(dji_motor_t *motor, uint8_t motor_type, can_channel_e can_periph, uint32_t id, float reduction_ratio);
void dji_motor_get_data(uint32_t id, uint8_t *data);
void dji_motor_set_torque(dji_motor_t *motor, float t);
void dji_motor_output_data(void);

#endif
