#include "filter.h"

/*
 * @brief     一阶低通滤波器初始化
 * @param[in] pft: 滤波器实例
 * @param[in] k  : 滤波系数
 * @retval    void
 */
void lpfo_filter_init(lpfo_filter_t *pft, float k)
{
    pft->init = 0;
    pft->k = k;
    pft->last_data = 0;
    pft->now_data = 0;
}

/*
 * @brief     一阶低通滤波器计算
 * @param[in] pft : 滤波器实例
 * @param[in] data: 需要滤波数据
 * @retval    滤波后数据
 * @note      第一次接收数据或者k为1时不滤波。
 */
float lpfo_filter_calc(lpfo_filter_t *pft, float data)
{
    if (pft->init == 0) {
        pft->now_data = data;
        pft->last_data = data;
        pft->init = 1;
    } else {
        pft->now_data = pft->k * pft->now_data + (1.0f - pft->k) * pft->last_data;
        pft->last_data = pft->now_data;
    }
    return pft->now_data;
}

/*
 * @brief     平均低通滤波器初始化
 * @param[in] pft       : 滤波器实例
 * @param[in] filter_num: 滤波数量
 * @retval    void
 */
void avg_filter_init(avg_filter_t *pft, float filter_num)
{
    pft->init = 0;
    pft->filter_num = filter_num;
    pft->index = 0;
}

/*
 * @brief     一阶低通滤波器计算
 * @param[in] pft : 滤波器实例
 * @param[in] data: 需要滤波数据
 * @retval    滤波后数据
 * @note      第一次接收数据或者k为1时不滤波。
 */
float avg_filter_calc(avg_filter_t *pft, float data)
{
    float sum, avg;
    if (pft->init == 0) {
        pft->array[pft->index++] = data;
        for (uint8_t i = 0; i < pft->index; i++)
            sum += pft->array[i];
        avg = sum / pft->index;
        if (pft->index == pft->filter_num) {
            pft->init = 1;
            pft->index = 0;
        }
        return avg;
    } else {
        if (pft->index == pft->filter_num)
            pft->index = 0;
        pft->array[pft->index++] = data;
        for (uint8_t i = 0; i < pft->filter_num; i++)
            sum += pft->array[i];
        avg = sum / pft->filter_num;
        return avg;
    }
}
