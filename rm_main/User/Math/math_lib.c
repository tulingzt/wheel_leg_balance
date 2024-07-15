#include "math_lib.h"
#include "stdint.h"
#include "math.h"

/*
 * @brief     数据限幅函数
 * @param[in] data: 限幅数据指针
 * @param[in] min : 最小值
 * @param[in] max : 最大值
 * @retval    void
 */
void data_limit(float *data, float min, float max)
{
    if (*data >= max)
        *data = max;
    else if (*data <= min)
        *data = min;
}

void abs_limit(float *data, float abs_max, float offset)
{
    if (*data > abs_max + offset)
        *data = abs_max + offset;
    else if (*data < -abs_max + offset)
        *data = -abs_max + offset;
}

/*
 * @brief     数据融合函数
 * @param[in] data1 : 数据1
 * @param[in] data2 : 数据2
 * @param[in] weight: 融合权重
 * @retval    融合后数据
 * @note      权重为0时取data1，权重为1时取data2
 */
float data_fusion(float data1, float data2, float weight)
{
    if (weight > 1)
        weight = 1;
    else if (weight < 0)
        weight = 0;
    return data1 * (1.0f - weight) + data2 * weight;
}

/*
 * @brief     sigmoid函数
 * @param[in] x: 输入值
 * @retval    void
 * @note      将(负无穷,正无穷)映射为(0,1),其中0映射为0.5
 */
float sigmoid_function(float x)
{
    return 1.0f / (1 + powf(E, -x));
}

/*
 * @brief     冒泡排序函数
 * @param[in] data: 排序数组指针
 * @param[in] len : 数组长度
 * @retval    void
 * @note      从大到小排序
 */
void bubble_sort(float *data, uint8_t len)
{
    float tmp;
    for (uint8_t i = 0; i < len; i++) {
        for (uint8_t j = 0; j < len - i - 1; j++) {
            if (data[j] < data[j+1]) {
                tmp = data[j];
                data[j] = data[j+1];
                data[j+1] = tmp;
            }
        }
    }
}

/*
 * @brief     环形数据计算偏差值
 * @param[in] set        : 目标值
 * @param[in] get        : 实际值
 * @param[in] circle_para: 一圈数值
 * @retval    返回目标值与实际值在环形中的差
 */
float circle_error(float set, float get, float circle_para)
{
    float error;
    if (set > get) {
        if (set - get > circle_para / 2) {
            error = set - get - circle_para;
        } else {
            error = set - get;
        }
    } else if (set < get) {
        if (set - get < -1.0f * circle_para / 2) {
            error = set - get + circle_para;
        } else {
            error = set - get;
        }
    } else {
        error = 0;
    }
    return error;
}

/*
 * @brief     斜波函数计算，无限幅和时间间隔
 * @param[in] ref  : 目标值
 * @param[in] fdb  : 实际值
 * @param[in] slope: 斜坡值
 * @retval    返回一个斜坡过的目标值
 */
float ramp_input(float ref, float fdb, float slope)
{
    if (ref - fdb > slope)
        return fdb + slope;
    else if (ref - fdb < -slope)
        return fdb - slope;
    else
        return ref;
}

/*
 * @brief     斜波函数初始化
 * @param[in] ramp        : 斜波结构体
 * @param[in] frame_period: 时间间隔
 * @param[in] min         : 斜坡最小值
 * @param[in] max         : 斜坡最大值
 * @retval    void
 */
void ramp_init(ramp_t *ramp, float frame_period, float min, float max)
{
    ramp->frame_period = frame_period;
    ramp->min = min;
    ramp->max = max;
    ramp->out = 0;
}

/*
 * @brief     斜波函数计算
 * @param[in] ramp : 斜波结构体
 * @param[in] input: 输入值
 * @retval    返回一个斜坡过的输出
 */
float ramp_calc(ramp_t *ramp, float target)
{
    ramp->target = target;
    if (ramp->out < ramp->target) {
        ramp->out += ramp->frame_period;
    } else if (ramp->out > ramp->target) {
        ramp->out -= ramp->frame_period;
    }
    data_limit(&(ramp->out), ramp->min, ramp->max); 
    return ramp->out;
}

/*
 * @brief     最小二乘法直线拟合
 * @param[in] x  : 拟合点横坐标数组
 * @param[in] y  : 拟合点纵坐标数组
 * @param[in] num: 数组长度
 * @param[in] a  : 拟合直线系数
 * @param[in] b  : 拟合直线系数
 * @retval    void
 * @note      y=ax+b
 */
void least_square_linear_fit(float x[], float y[], const int num, float *a, float *b)
{
    float sum_x2 = 0, sum_y = 0, sum_x = 0, sum_xy = 0;
    for (int i = 0; i < num; i++) {
        sum_x2 += x[i] * x[i];
        sum_y += y[i];
        sum_x += x[i];
        sum_xy += x[i] * y[i];
    }
    *a = (num * sum_xy - sum_x * sum_y)/(num * sum_x2 - sum_x * sum_x);
    *b = (sum_x2 * sum_y - sum_x * sum_xy)/(num * sum_x2 - sum_x * sum_x);
}

/*
 * @brief     复数辐角计算
 * @param[in] x: 复数x坐标
 * @param[in] y: 复数y坐标
 * @retval    复数辐角(-PI,PI] (rad),零向量辐角默认为0
 */
float vector_arg(float x, float y)
{
    if (x == 0) {
        if (y < 0) {
            return -PI/2;
        } else if (y == 0) {
            return 0;
        } else {
            return PI/2;
        }
    } else if (x < 0) {
        if (y < 0) {
            return atanf(y/x) - PI;
        } else {
            return atanf(y/x) + PI;
        }
    } else {
        return atanf(y/x);
    }
}
