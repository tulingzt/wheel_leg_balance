#ifndef __MATH_CALCU_H
#define __MATH_CALCU_H

#include "stdint.h"

#ifndef PI
    #define PI 3.14159265358979323846f
#endif
#define E  2.718282f
#define GRAVITY 9.81f
#define SIGN(x) ((x)>0?(1):((x)<0?(-1) 0))
#define ABS(x) ((x>0)?(x):(-(x)))
#define ROUND(type, x) ((type)((x)>0?(x)+0.5f:(x)-0.5f))

typedef struct
{
    float target;
    float out;
    float min, max;
    float frame_period;
} ramp_t;

void data_limit(float *data, float min, float max);
void abs_limit(float *data, float abs_max, float offset);
float data_fusion(float data1, float data2, float weight);
float sigmoid_function(float x);
void bubble_sort(float *data, uint8_t len);
float circle_error(float set, float get, float circle_para);
float ramp_input(float ref, float fdb, float slope);
void ramp_init(ramp_t *ramp, float frame_period, float min, float max);
float ramp_calc(ramp_t *ramp, float input);
void least_square_linear_fit(float x[], float y[], const int num, float *a, float *b);
float vector_arg(float x, float y);

#endif
