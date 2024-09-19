#ifndef __KALMAN_FILTERS_H
#define __KALMAN_FILTERS_H

#include "stdint.h"
#include "stdlib.h"
#include "arm_math.h"

#ifndef user_malloc
    #ifdef _CMSIS_OS_H
        #define user_malloc pvPortMalloc
    #else
        #define user_malloc malloc
    #endif
#endif

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef struct kf_t
{
    //向量长度
    uint8_t x_size, u_size, z_size;
    //计算状态
    int8_t mat_status;
    //矩阵结构体
    mat xhat, xhatminus;//状态向量 x(k|k) x(k|k-1)
    mat u;              //控制向量 u
    mat z;              //测量向量 z
    mat P, Pminus;      //协方差矩阵 P(k|k) P(k|k-1)
    mat A, AT;          //状态转移矩阵 A AT
    mat B;              //控制转移矩阵 B
    mat H, HT;          //测量转移向量 H HT
    mat Q;              //过程噪声协方差矩阵 Q
    mat R;              //测量噪声协方差矩阵 R
    mat K;              //卡尔曼增益 K
    mat S, temp_matrix1, temp_matrix2, temp_vector1, temp_vector2;//中间变量
    //矩阵存储空间指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *A_data, *AT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data1, *temp_matrix_data2, *temp_vector_data1, *temp_vector_data2;
    //外部数据接口
    float *filter_vector, *measured_vector, *control_vector;
    float *min_variance;      //最小方差 避免方差过度收敛
    //配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t skip_eq1, skip_eq2, skip_eq3, skip_eq4, skip_eq5;
    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*user_func0_f)(struct kf_t *kf);
    void (*user_func1_f)(struct kf_t *kf);
    void (*user_func2_f)(struct kf_t *kf);
    void (*user_func3_f)(struct kf_t *kf);
    void (*user_func4_f)(struct kf_t *kf);
    void (*user_func5_f)(struct kf_t *kf);
    void (*user_func6_f)(struct kf_t *kf);
} kalman_filter_t;

void kalman_filter_init(kalman_filter_t *kf, uint8_t x_size, uint8_t u_size, uint8_t z_size);
void *kalman_filter_update(kalman_filter_t *kf);

#endif
