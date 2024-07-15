#include "drv_ht_motor.h"

#define LIMIT_MIN_MAX(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

static list_t object_list = {&object_list, &object_list};

ht_motor_t joint_motor[4];

//根据协议，对float参数进行转换，用于数据发送
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

//根据协议，对uint参数进行转换，用于数据接收
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//static void ht_motor_output_single_data(ht_motor_t *motor);

/*
 * @brief     海泰电机初始化设置
 * @param[in] motor     : 电机数据结构体
 * @param[in] can_periph: 电机所在can通道
 * @param[in] id        : 电机id
 * @param[in] zero_point: 电机安装零点
 * @retval    void
 */
void ht_motor_init(ht_motor_t *motor, can_channel_e can_channel, uint32_t id, float zero_point)
{
    motor->can_channel = can_channel;
    motor->can_id = id;
    motor->zero_point = zero_point;
    ht_motor_set_control_cmd(motor, CMD_MOTOR_MODE);
    HAL_Delay(1);
    ht_motor_set_control_para(motor, 0, 0, 0, 0, 0);
    ht_motor_output_single_data(motor);
    HAL_Delay(1);
    list_add(&(motor->list), &object_list);
}

/*
 * @brief     接收海泰电机数据，并化为国际单位
 * @param[in] motor: 电机数据结构体
 * @param[in] data : 数据指针
 * @retval    void
 */
static void ht_motor_get_single_data(ht_motor_t *motor, uint8_t *data)
{
    uint16_t tmp_value;
    motor->receive_cnt++;
    motor->err_percent = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
    //根据协议，对uint参数进行转换
    tmp_value = (data[1] << 8) | data[2];
    motor->position = uint_to_float(tmp_value, P_MIN, P_MAX, 16);
    tmp_value = (data[3] << 4) | (data[4] >> 4);
    motor->velocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
    tmp_value = ((0x0f & data[4]) << 8) | data[5];
    motor->torque = uint_to_float(tmp_value, T_MIN, T_MAX, 12);
}

void ht_motor_get_data(uint8_t id, uint8_t *data)
{
    list_t *node = NULL;
    ht_motor_t *object;
    for (node = object_list.next; node != &(object_list); node = node->next) {
        object = list_entry(node, ht_motor_t, list);
        if (object->can_id == id) {
            ht_motor_get_single_data(object, data);
        }
    }
}

void ht_motor_set_control_para(ht_motor_t *motor, float p, float v, float kp, float kd, float t)
{
    motor->p = p;
    motor->v = v;
    motor->kp = kp;
    motor->kd = kd;
    motor->t = t;
    //限制输入的参数在定义的范围内
    LIMIT_MIN_MAX(motor->p, P_MIN, P_MAX);
    LIMIT_MIN_MAX(motor->v, V_MIN, V_MAX);
    LIMIT_MIN_MAX(motor->kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(motor->kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(motor->t, T_MIN, T_MAX);
}

void ht_motor_set_control_cmd(ht_motor_t *motor, uint8_t cmd)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
        case CMD_MOTOR_MODE:
            buf[7] = 0xFC;
            break;
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        default:
        return; /* 直接退出函数 */
    }
    can_std_transmit(motor->can_channel, motor->can_id, buf);
}

void ht_motor_output_single_data(ht_motor_t *motor)
{
    static uint8_t buf[8];
    uint16_t p, v ,kp, kd, t;
//    //限制输入的参数在定义的范围内
    LIMIT_MIN_MAX(motor->p, P_MIN, P_MAX);
    LIMIT_MIN_MAX(motor->v, V_MIN, V_MAX);
    LIMIT_MIN_MAX(motor->kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(motor->kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(motor->t, T_MIN+1, T_MAX-1);
    //根据协议，对float参数进行转换
    p = float_to_uint(motor->p, P_MIN, P_MAX, 16);
    v = float_to_uint(motor->v, V_MIN, V_MAX, 12);
    kp = float_to_uint(motor->kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(motor->kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(motor->t, T_MIN+1, T_MAX-1, 12);
    //根据传输协议，把数据转换为CAN命令数据字段
    buf[0] = p >> 8;
    buf[1] = p & 0xFF;
    buf[2] = v >> 4;
    buf[3] = ((v & 0x0F) << 4) | (kp >> 8);
    buf[4] = kp & 0xFF;
    buf[5] = kd >> 4;
    buf[6] = ((kd & 0x0F) << 4) | (t >> 8);
    buf[7] = t & 0xFF;
    //通过CAN接口把buf中的内容发送出去
    motor->send_cnt++;
    can_std_transmit(motor->can_channel, motor->can_id, buf);
}

/*
 * @brief  将各电机数据发送
 * @retval void
 */
void ht_motor_output_data(void)
{
    list_t *node = NULL;
    ht_motor_t *object;
    for (node = object_list.next; node != &(object_list); node = node->next) {
        object = list_entry(node, ht_motor_t, list);
        ht_motor_output_single_data(object);
    }
}
