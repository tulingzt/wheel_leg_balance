#include "leg_vmc.h"
#include "math_matrix.h"
#include "arm_math.h"

vmc_t vmc[2];

/*
 * @brief     五连杆vmc解算初始化
 * @param[in] v : vmc实例
 * @param[in] legLength: 五连杆各长度
 * @retval    void
 */
void vmc_init(vmc_t* v, const float legLength[5])
{
    for(int i = 1; i < 6; i++)
        v->L[i] = legLength[i-1];
    //写出髋部中点
    v->mp_fdb.xm = v->L[5] / 2;
    v->mp_fdb.ym = 0;
    v->mp_ref.xm = v->L[5] / 2;
    v->mp_ref.ym = 0;
}

/*
 * @brief     五连杆vmc正解算
 * @param[in] v : vmc实例
 * @param[in] q1 q4: 实际杆角度      解算得到虚拟杆长度和角度
 * @param[in] w1 w4: 实际杆角速度    解算得到虚拟杆伸长速度和角速度
 * @param[in] t1 t4: 实际杆力矩      解算得到虚拟杆转动力矩和支持力
 * @retval    void
 */
void vmc_forward_solution(vmc_t* v, float q1, float q4, float w1, float w4, float t1, float t4)
{
    //------------------------位置运动学正解算------------------------//
    //反馈关节角度
    v->q_fdb[1] = q1;
    v->q_fdb[4] = q4;
    //计算膝关节坐标
    v->mp_fdb.xb = v->L[1] * arm_cos_f32(q1);
    v->mp_fdb.yb = v->L[1] * arm_sin_f32(q1);
    v->mp_fdb.xd = v->L[4] * arm_cos_f32(q4) + v->L[5];
    v->mp_fdb.yd = v->L[4] * arm_sin_f32(q4);
    //计算解算所需中间系数
    v->mp_fdb.Lbd = sqrtf(powf(v->mp_fdb.xd - v->mp_fdb.xb, 2) + powf(v->mp_fdb.yd - v->mp_fdb.yb, 2));
    v->mp_fdb.A0 = 2 * v->L[2] * (v->mp_fdb.xd - v->mp_fdb.xb);
    v->mp_fdb.B0 = 2 * v->L[2] * (v->mp_fdb.yd - v->mp_fdb.yb);
    v->mp_fdb.C0 = powf(v->L[2], 2) + powf(v->mp_fdb.Lbd, 2) - powf(v->L[3], 2);
    //解算左膝关节角
    v->q_fdb[2] = 2 * atan2f(v->mp_fdb.B0 + sqrtf(powf(v->mp_fdb.A0, 2) + powf(v->mp_fdb.B0, 2) \
                    - powf(v->mp_fdb.C0, 2)), v->mp_fdb.A0 + v->mp_fdb.C0);
    //得到足端坐标
    v->mp_fdb.xc = v->mp_fdb.xb + v->L[2] * arm_cos_f32(v->q_fdb[2]);
    v->mp_fdb.yc = v->mp_fdb.yb + v->L[2] * arm_sin_f32(v->q_fdb[2]);
    //解算右膝关节角
    v->q_fdb[3] = atan2f(v->mp_fdb.yc - v->mp_fdb.yd, v->mp_fdb.xc - v->mp_fdb.xd);
    //计算虚拟腿连杆角度与长度（虚拟模型）
    v->L_fdb = sqrtf(powf(v->mp_fdb.xc - v->mp_fdb.xm, 2) + powf(v->mp_fdb.yc - v->mp_fdb.ym, 2));
    v->q_fdb[0] = atan2f(v->mp_fdb.yc - v->mp_fdb.ym, v->mp_fdb.xc - v->mp_fdb.xm);
    
    //------------------------速度运动学正解算 ------------------------//
    //反馈关节角速度
    v->W_fdb.e.w1_fdb = w1;
    v->W_fdb.e.w4_fdb = w4;
    //写出角速度正解算矩阵 [w0 vy0] = Jwv [w1 w4]
    v->Jwv.e.x11 = ( v->L[1] * arm_sin_f32(v->q_fdb[0]) * arm_sin_f32(v->q_fdb[3]) * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) \
                    - v->L[4] * arm_cos_f32(v->q_fdb[0]) * arm_sin_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4])) \
                    / arm_sin_f32(v->q_fdb[2] - v->q_fdb[3]);
    v->Jwv.e.x12 = ( v->L[1] * arm_cos_f32(v->q_fdb[0]) * arm_sin_f32(v->q_fdb[3]) * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) \
                    + v->L[4] * arm_sin_f32(v->q_fdb[0]) * arm_sin_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4])) \
                    / arm_sin_f32(v->q_fdb[2] - v->q_fdb[3]);
    v->Jwv.e.x21 = (-v->L[1] * arm_sin_f32(v->q_fdb[0]) * arm_cos_f32(v->q_fdb[3]) * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) \
                    + v->L[4] * arm_cos_f32(v->q_fdb[0]) * arm_cos_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4])) \
                    / arm_sin_f32(v->q_fdb[2] - v->q_fdb[3]);
    v->Jwv.e.x22 = (-v->L[1] * arm_cos_f32(v->q_fdb[0]) * arm_cos_f32(v->q_fdb[3]) * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) \
                    - v->L[4] * arm_sin_f32(v->q_fdb[0]) * arm_cos_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4])) \
                    / arm_sin_f32(v->q_fdb[2] - v->q_fdb[3]);
    //速度正解算
    aMartix_Mul(v->Jwv.array, v->W_fdb.array, v->V_fdb.array, 2, 2, 1);
    v->V_fdb.e.w0_fdb *= -1.0f / v->L_fdb; //线速度转换为角速度
    //腿长二阶导数(差分)解算
    v->Acc_fdb.L0_ddot = (v->V_fdb.e.vy0_fdb - v->Acc_fdb.vy0_fdb_last) / (2 * 0.001f);
    v->Acc_fdb.vy0_fdb_last = v->V_fdb.e.vy0_fdb;
    
    //------------------------动力学正解算------------------------//
    //反馈关节力矩
    v->T_fdb.e.T1_fdb = t1;
    v->T_fdb.e.T4_fdb = t4;
    //写出力的正解算矩阵 [T0 Fy] = Jtf [T1 T4]
    v->Jtf.e.x11 = v->L_fdb * arm_sin_f32(v->q_fdb[0] - v->q_fdb[2]) \
                    / (v->L[1] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]));
    v->Jtf.e.x12 = v->L_fdb * arm_sin_f32(v->q_fdb[0] - v->q_fdb[3]) \
                    / (v->L[4] * arm_sin_f32(v->q_fdb[4] - v->q_fdb[3]));
    v->Jtf.e.x21 = arm_cos_f32(v->q_fdb[0] - v->q_fdb[2]) \
                    / (v->L[1] * arm_sin_f32(v->q_fdb[2] - v->q_fdb[1]));
    v->Jtf.e.x22 = arm_cos_f32(v->q_fdb[0] - v->q_fdb[3]) \
                    / (v->L[4] * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4]));
    //力正解算
    aMartix_Mul(v->Jtf.array, v->T_fdb.array, v->F_fdb.array, 2, 2, 1);
}

/*
 * @brief     五连杆vmc逆解算
 * @param[in] v : vmc实例
 * @param[in] L_ref q0_ref: 虚拟杆长度和角度实际杆角度   解算得到实际杆角度
 * @param[in] T0 Fy:        虚拟杆转动力矩和支持力       解算得到实际杆力矩
 * @retval    void
 */
void vmc_inverse_solution(vmc_t* v, float L_ref, float q0_ref, float T0, float Fy)
{
    //------------------------位置运动学逆解算------------------------//
    //目标关节角度
    v->L_ref = L_ref;
    v->q_ref[0] = q0_ref;
    //得到足端坐标
    v->mp_ref.xc = v->mp_ref.xm + v->L_ref * arm_cos_f32(v->q_ref[0]);
    v->mp_ref.yc = v->mp_ref.ym + v->L_ref * arm_sin_f32(v->q_ref[0]);
    //解算大小腿间角度
    //[0 pi]
    v->mp_ref.q12 = PI - acosf((powf(v->mp_ref.xc, 2) + powf(v->mp_ref.yc, 2) - powf(v->L[1], 2)\
                    - powf(v->L[2], 2)) / (2 * v->L[1] * v->L[2]));
    //[pi 2pi]
    v->mp_ref.q34 = PI + acosf((powf(v->mp_ref.xc - v->L[5], 2) + powf(v->mp_ref.yc, 2) - powf(v->L[3], 2)\
                    - powf(v->L[4], 2)) / (2 * v->L[3] * v->L[4]));
    //计算解算所需中间系数 得到大腿与水平面角度
    v->mp_ref.A = v->mp_ref.xc;
    v->mp_ref.B = v->L[2] * arm_sin_f32(v->mp_ref.q12);
    v->mp_ref.C = v->L[1] - v->L[2] * arm_cos_f32(v->mp_ref.q12);
    v->mp_ref.q01 = acosf(v->mp_ref.C / sqrtf(powf(v->mp_ref.B, 2) + powf(v->mp_ref.C, 2)))\
                    + acosf(v->mp_ref.A / sqrtf(powf(v->mp_ref.B, 2) + powf(v->mp_ref.C, 2)));
    v->mp_ref.A = v->L[5] - v->mp_ref.xc;
    v->mp_ref.B = v->L[3] * arm_sin_f32(v->mp_ref.q34);
    v->mp_ref.C = v->L[4] - v->L[3] * arm_cos_f32(v->mp_ref.q34);
    v->mp_ref.q40 = acosf(v->mp_ref.C / sqrtf(powf(v->mp_ref.B, 2) + powf(v->mp_ref.C, 2)))\
                    + acosf(v->mp_ref.A / sqrtf(powf(v->mp_ref.B, 2) + powf(v->mp_ref.C, 2)));
    //得到目标各角度
    v->q_ref[1] = v->mp_ref.q01;
    v->q_ref[2] = v->mp_ref.q12 + v->mp_ref.q01 - PI;
    v->q_ref[3] = v->mp_ref.q34 - v->mp_ref.q40;
    v->q_ref[4] = PI - v->mp_ref.q40;
    
    //------------------------动力学逆解算------------------------//
    //目标关节力矩
    v->F_ref.e.T0_ref = T0;
    v->F_ref.e.Fy_ref = Fy;
    //写出力的正解算矩阵 [T1 T4] = Jft [T0 Fy]
    v->Jft.e.x11 = (v->L[1] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) * arm_cos_f32(v->q_fdb[0] - v->q_fdb[3])) \
                    / (v->L_fdb * arm_sin_f32(v->q_fdb[3] - v->q_fdb[2]));
    v->Jft.e.x12 = (v->L[1] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]) * arm_sin_f32(v->q_fdb[0] - v->q_fdb[3])) \
                    / arm_sin_f32(v->q_fdb[3] - v->q_fdb[2]);
    v->Jft.e.x21 = (v->L[4] * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4]) * arm_cos_f32(v->q_fdb[0] - v->q_fdb[2])) \
                    / (v->L_fdb * arm_sin_f32(v->q_fdb[3] - v->q_fdb[2]));
    v->Jft.e.x22 = (v->L[4] * arm_sin_f32(v->q_fdb[3] - v->q_fdb[4]) * arm_sin_f32(v->q_fdb[0] - v->q_fdb[2])) \
                    / arm_sin_f32(v->q_fdb[3] - v->q_fdb[2]);
    //力逆解算
    aMartix_Mul(v->Jft.array, v->F_ref.array, v->T_ref.array, 2, 2, 1); 
}
