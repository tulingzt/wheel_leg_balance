#ifndef __PID_H
#define __PID_H

typedef enum
{
    NONE = 0x00,
    CHANG_I_RATE = 0x01
} pid_mode_e;

typedef struct
{
    float kp, ki, kd;
    float threshold_a, threshold_b;//变速积分 a < err < a+b
    float i_item;
    float i_max, out_max;
    float ref, fdb;
    float err[2];
    float p_out, i_out, d_out, output;
    pid_mode_e mode;
} pid_t;

typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float ref;
    float last_ref;

    float lpf_rc; // RC = 1/omegac
    float dt;

    float ref_dot;
    float ref_ddot;
    float last_ref_dot;

    float output;
    float out_max;
} feed_forward_t;

void pid_init(pid_t *pid, pid_mode_e mode, float kp, float ki, float kd, float i_max, float out_max);
float pid_calc(pid_t *pid, float ref, float fdb);

void feed_forward_init(feed_forward_t *ffc, float dt, float max_out, float *c, float lpf_rc);
float feed_forward_calc(feed_forward_t *ffc, float ref);

#endif
