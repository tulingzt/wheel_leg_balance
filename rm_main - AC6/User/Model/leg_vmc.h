#ifndef __LEG_VMC_H
#define __LEG_VMC_H

typedef struct
{
    float L[6];//连杆长度int[1,6]，L[0]不用
    //------------------------位置的正运动学解算------------------------//
    //Input:  q_fdb[1], q_fdb[4]
    //Output: q_fdb[0], L_fdb
    float L_fdb;//等效连杆长度
    float q_fdb[5];//关节角度int[0,4]，q[0]为等效连杆角度
    //中间变量
    struct
    {
        float xb, yb, xd, yd, xc, yc, xm, ym;
        float Lbd;
        float A0, B0, C0;
    } mp_fdb;
    //------------------------速度的正运动学解算------------------------//
    //Input:  w1_fdb, w4_fdb
    // V_fdb = Jwv * W_fdb
    //Output: w0_fdb, vy0_fdb
    union
    {
        float array[2 * 1];
        struct
        {
            float w1_fdb;
            float w4_fdb;
        } e;
    } W_fdb;//关节电机反馈角速度
    union
    {
        float array[2 * 1];
        struct
        {
            float w0_fdb;
            float vy0_fdb;
        } e;
    } V_fdb;//足端角速度和速度反馈
    union
    {
        float array[2 * 2];
        struct
        {
            float x11;
            float x12;
            float x21;
            float x22;
        } e;
    } Jwv;
    struct
    {
        float vy0_fdb_last;
        float L0_ddot;
    } Acc_fdb;//腿长微分
    //------------------------力的正运动学解算------------------------//
    //Input:  T1_fdb, T4_fdb
    // F_fdb = Jtf * T_fdb
    //output: T0_fdb, Fy_fdb
    union
    {
        float array[2 * 1];
        struct
        {
            float T1_fdb;
            float T4_fdb;
        } e;
    } T_fdb;//关节电机反馈力矩
    union
    {
        float array[2 * 1];
        struct
        {
            float T0_fdb;
            float Fy_fdb;
        } e;
    } F_fdb;//换算后的足端力和力矩
    union
    {
        float array[2 * 2];
        struct
        {
            float x11;
            float x12;
            float x21;
            float x22;
        } e;
    } Jtf;
    //------------------------位置的逆运动学解算------------------------//
    //Input:  L_ref, q_ref[0]
    //output: q_ref[1], q_ref[4]
    float L_ref;//腿长设定值
    float q_ref[5];//关节角度设定值
    //中间变量
    struct
    {
        float xc, yc, xm, ym;
        float q01, q12, q34, q40;
        float A, B, C;
    } mp_ref;
    //------------------------ 力的逆运动学解算 ------------------------//
    //Input:  T0_ref, Fy_ref
    // T_ref = Jft * F_ref
    //output: T1_ref, T4_ref
    union
    {
        float array[2 * 1];
        struct
        {
            float T0_ref;
            float Fy_ref;
        } e;
    } F_ref;//足端期望力和力矩  足端期望力由重力补偿+PID组成
    union
    {
        float array[2 * 1];
        struct
        {
            float T1_ref;
            float T4_ref;
        } e;
    } T_ref;//关节电机期望力和力矩
    union
    {
        float array[2 * 2];
        struct
        {
            float x11;
            float x12;
            float x21;
            float x22;
        } e;
    } Jft;
} vmc_t;

extern vmc_t vmc[2];

void vmc_init(vmc_t* vmc, const float legLength[5]);
void vmc_forward_solution(vmc_t* v, float q1, float q4, float w1, float w4, float t1, float t4);
void vmc_inverse_solution(vmc_t* v, float L_ref, float q0_ref, float T0, float Fy);

#endif
