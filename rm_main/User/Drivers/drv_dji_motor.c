#include "drv_dji_motor.h"
#include "string.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define LIMIT(x,limit) (x)=(((x)<=(-limit))?(-limit):(((x)>=(limit))?(limit):(x)))
#define DATA_RANGE 0
#define CURRENT_RANGE 1
#define TORQUE_CONSTANT 2
#define REDUCTION_RATIO 3

static list_t object_list = {&object_list, &object_list};
static uint8_t motor_send_flag[CAN_CHANNEL_NUM][4] = {0};
can_std_msg_t motor_msg[CAN_CHANNEL_NUM][4];

dji_motor_t fric_motor[2];
dji_motor_t driver_motor[2];
dji_motor_t pit_motor, yaw_motor;
dji_motor_t trigger_motor;

static const float motor_para_table[3][4] =
{
    //发送数据范围            发送数据所代表的物理量范围       力矩常数                        电机默认减速比
    {DJI_2006_MOTOR_DATA_RANGE, DJI_2006_MOTOR_CURRENT_RANGE, DJI_2006_MOTOR_TORQUE_CONSTANT, DJI_2006_ORIGINAL_REDUCTION_RATIO},
    {DJI_3508_MOTOR_DATA_RANGE, DJI_3508_MOTOR_CURRENT_RANGE, DJI_3508_MOTOR_TORQUE_CONSTANT, DJI_3508_ORIGINAL_REDUCTION_RATIO},
    {DJI_6020_MOTOR_DATA_RANGE, DJI_6020_MOTOR_CURRENT_RANGE, DJI_6020_MOTOR_TORQUE_CONSTANT, DJI_6020_ORIGINAL_REDUCTION_RATIO},
};

/*
 * @brief     大疆电机初始化设置
 * @param[in] motor          : 电机数据结构体
 * @param[in] motor_type     : 电机类型
 * @param[in] can_periph     : 电机所在can通道
 * @param[in] id             : 电机id
 * @param[in] reduction_ratio: 电机实际减速比
 * @retval    void
 */
void dji_motor_init(dji_motor_t *motor, uint8_t motor_type, can_channel_e can_channel, uint32_t id, float reduction_ratio)
{
    motor->motor_type = motor_type;
    motor->can_channel = can_channel;
    motor->can_id = id;
    motor->reduction_ratio = reduction_ratio;
    list_add(&(motor->list), &object_list);
}

/*
 * @brief     接收大疆电机数据，并化为国际单位
 * @param[in] motor: 电机数据结构体
 * @param[in] data : 数据指针
 * @retval    void
 */
static void dji_motor_get_single_data(dji_motor_t * motor, uint8_t *data)
{
    motor->receive_cnt++;
    motor->err_percent  = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
    motor->last_ecd     = motor->ecd;
    motor->ecd          = (uint16_t)(data[0] << 8 | data[1]);
    motor->speed_rpm    = (int16_t)(data[2] << 8 | data[3]);
    motor->rx_current   = (int16_t)(data[4] << 8 | data[5]);
    motor->temperature  = data[6];
    if (motor->receive_cnt < 50) {
        motor->offset_ecd = motor->ecd;
        motor->round_cnt = 0;
    }
    if (motor->ecd - motor->last_ecd >= 4096)
        motor->round_cnt--;
    else if (motor->ecd - motor->last_ecd <= -4096)
        motor->round_cnt++;
    motor->total_ecd = motor->round_cnt * 8192 + motor->ecd - motor->offset_ecd;
    
    motor->position = 2.0f * PI * motor->total_ecd / 8192 / motor->reduction_ratio;
    motor->velocity = 2.0f * PI * motor->speed_rpm / 60 / motor->reduction_ratio;
    motor->torque = (float)motor->rx_current \
                    * motor->reduction_ratio \
                    / motor_para_table[motor->motor_type][DATA_RANGE] \
                    * motor_para_table[motor->motor_type][CURRENT_RANGE] \
                    * motor_para_table[motor->motor_type][TORQUE_CONSTANT] \
                    / motor_para_table[motor->motor_type][REDUCTION_RATIO];
}

/*
 * @brief     根据id查找对应电机并接收数据
 * @param[in] id  : 对应id
 * @param[in] data: 数据指针
 * @retval    void
 */
void dji_motor_get_data(can_channel_e can_periph, uint32_t id, uint8_t *data)
{
    list_t *node = NULL;
    dji_motor_t *object;
    memset(&motor_msg, 0, sizeof(motor_msg));
    for (node = object_list.next; node != &(object_list); node = node->next) {
        object = list_entry(node, dji_motor_t, list);
        if (object->can_id == id && object->can_channel == can_periph) {
            dji_motor_get_single_data(object, data);
        }
    }
}

/*
 * @brief     大疆电机设置控制力矩，并转换成转矩电流
 * @param[in] motor: 电机数据结构体
 * @param[in] t    : 设置力矩
 * @retval    void
 */
void dji_motor_set_torque(dji_motor_t *motor, float t)
{
    motor->t = t;
    LIMIT(motor->t,   motor->reduction_ratio \
                    * motor_para_table[motor->motor_type][CURRENT_RANGE] \
                    * motor_para_table[motor->motor_type][TORQUE_CONSTANT] \
                    / motor_para_table[motor->motor_type][REDUCTION_RATIO]);
    motor->tx_current = (int16_t)(motor->t \
                        / motor->reduction_ratio \
                        * motor_para_table[motor->motor_type][DATA_RANGE] \
                        / motor_para_table[motor->motor_type][CURRENT_RANGE] \
                        / motor_para_table[motor->motor_type][TORQUE_CONSTANT] \
                        * motor_para_table[motor->motor_type][REDUCTION_RATIO]);
}

/*
 * @brief  将各电机数据整理等待发送
 * @retval void
 */
static void dji_motor_fill_data(void)
{
    list_t *node = NULL;
    dji_motor_t *object;
    memset(&motor_msg, 0, sizeof(motor_msg));
    for (node = object_list.next; node != &(object_list); node = node->next) {
        object = list_entry(node, dji_motor_t, list);
        if (object->motor_type == DJI_6020_MOTOR) {
            if (object->can_id < 0x209){
                motor_msg[object->can_channel][2].id = 0x1FE;
                motor_msg[object->can_channel][2].data[(object->can_id - 0x205) * 2] = object->tx_current >> 8;
                motor_msg[object->can_channel][2].data[(object->can_id - 0x205) * 2 + 1] = object->tx_current;
                motor_send_flag[object->can_channel][2] = 1;
            } else {
                motor_msg[object->can_channel][3].id = 0x2FE;
                motor_msg[object->can_channel][3].data[(object->can_id - 0x209) * 2] = object->tx_current >> 8;
                motor_msg[object->can_channel][3].data[(object->can_id - 0x209) * 2 + 1] = object->tx_current;
                motor_send_flag[object->can_channel][3] = 1;
            }
        } else {
            if (object->can_id < 0x205) {
                motor_msg[object->can_channel][0].id = 0x200;
                motor_msg[object->can_channel][0].data[(object->can_id - 0x201) * 2] = object->tx_current >> 8;
                motor_msg[object->can_channel][0].data[(object->can_id - 0x201) * 2 + 1] = object->tx_current;
                motor_send_flag[object->can_channel][0] = 1;
            } else {
                motor_msg[object->can_channel][1].id = 0x1FF;
                motor_msg[object->can_channel][1].data[(object->can_id - 0x205) * 2] = object->tx_current >> 8;
                motor_msg[object->can_channel][1].data[(object->can_id - 0x205) * 2 + 1] = object->tx_current;
                motor_send_flag[object->can_channel][1] = 1;
            }
        }
        object->send_cnt++;
    }
}

/*
 * @brief  将各电机数据发送
 * @retval void
 */
void dji_motor_output_data(void)
{
    dji_motor_fill_data();
    for (can_channel_e can_channel = CAN_CHANNEL_1; can_channel != CAN_CHANNEL_NUM; can_channel++) {
        for (int i = 0; i < 4; i++) {
            if (motor_send_flag[can_channel][i] == 1) {
                    can_std_transmit(can_channel, motor_msg[can_channel][i].id, motor_msg[can_channel][i].data);
                    motor_send_flag[can_channel][i] = 0;
                }
        }
    }
}
