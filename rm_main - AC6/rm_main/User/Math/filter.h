#ifndef __FILTER_H
#define __FILTER_H

#include "stdint.h"

//一阶低通滤波器
typedef struct
{
    uint8_t init;
    float k;
    float last_data, now_data;
} lpfo_filter_t;

void lpfo_filter_init(lpfo_filter_t *pft, float k);
float lpfo_filter_calc(lpfo_filter_t *pft, float data);

//平均滤波器
#define AVG_FILTER_MAX_NUM 10
typedef struct
{
    uint8_t init;
    uint8_t index, filter_num;
    float array[AVG_FILTER_MAX_NUM];
} avg_filter_t;

void avg_filter_init(avg_filter_t *pft, float filter_num);
float avg_filter_calc(avg_filter_t *pft, float data);

#endif
