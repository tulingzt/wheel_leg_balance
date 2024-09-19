#include "pid.h"

#define LIMIT(x,limit) (x)=(((x)<=(-limit))?(-limit):(((x)>=(limit))?(limit):(x)))
#define ABS(x) ((x>0)?(x):(-x))

/*
 * @brief  pid初始化
 * @retval void
 */
void pid_init(pid_t *pid, pid_mode_e mode, float kp, float ki, float kd, float i_max, float out_max)
{
    pid->mode = mode;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
}

static void f_changing_intergration_rate(pid_t *pid)
{
    if (pid->err[0] * pid->i_out > 0) {
        if (ABS(pid->err[0]) <= pid->threshold_a)
            return;
        else if (ABS(pid->err[0]) <= (pid->threshold_a + pid->threshold_b))
            pid->i_item *= (pid->threshold_b - ABS(pid->err[0]) + pid->threshold_a) / pid->threshold_b;
        else
            pid->i_item = 0;
    }
}

/*
 * @brief     pid计算
 * @param[in] pid: pid结构体
 * @param[in] ref: 期望值
 * @param[in] fdb: 反馈值
 * @retval    pid计算结果
 */
float pid_calc(pid_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;
    
    pid->p_out = pid->kp * pid->err[0];
    pid->i_item = pid->ki * (pid->err[0] + pid->err[1]) / 2;
    pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
    
    if (pid->mode & CHANG_I_RATE)
        f_changing_intergration_rate(pid);
    
    pid->i_out += pid->i_item;
    LIMIT(pid->i_out, pid->i_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT(pid->output, pid->out_max);
    return pid->output;
}

/*
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void feed_forward_init(feed_forward_t *ffc, float dt, float max_out, float *c, float lpf_rc)
{
    ffc->out_max = max_out;
    ffc->dt = dt;
    // 设置前馈控制器参数 详见前馈控制结构体定义
    if (c != 0 && ffc != 0) {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    } else {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->out_max = 0;
    }
    ffc->lpf_rc = lpf_rc;
    ffc->output = 0;
}

/*
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float feed_forward_calc(feed_forward_t *ffc, float ref)
{
    ffc->ref = ref * ffc->dt / (ffc->lpf_rc + ffc->dt) +
               ffc->ref * ffc->lpf_rc / (ffc->lpf_rc + ffc->dt);

    // 计算一阶导数
    // calculate first derivative
    ffc->ref_dot = (ffc->ref - ffc->last_ref) / ffc->dt;

    // 计算二阶导数
    // calculate second derivative
    ffc->ref_ddot = (ffc->ref_dot - ffc->last_ref_dot) / ffc->dt;

    // 计算前馈控制输出
    // calculate feed-forward controller output
    ffc->output = ffc->c[0] * ffc->ref + ffc->c[1] * ffc->ref_dot + ffc->c[2] * ffc->ref_ddot;

    LIMIT(ffc->output, ffc->out_max);

    ffc->last_ref = ffc->ref;
    ffc->last_ref_dot = ffc->ref_dot;

    return ffc->output;
}
