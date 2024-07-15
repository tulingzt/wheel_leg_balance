#include "wlr.h"
#include "chassis_task.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "prot_imu.h"
#include "drv_dji_motor.h"
#include "prot_dr16.h"
#include "pid.h"
#include "kalman_filter.h"
#include "math_lib.h"
#include "math_matrix.h"

#define WLR_SIGN(x) ((x) > 0? (1): (-1))

#define CHASSIS_PERIOD_DU 2

const float LegLengthParam[5] = {0.150f, 0.270f, 0.270f, 0.150f, 0.150f};
float mb = 7.75f;
//float mb = 4.4f;//原先值
float ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.51f;//两轮间距
const float WheelRadius = 0.065f;//轮子半径
const float LegLengthMax = 0.35f, LegLengthMin = 0.11f;

const float LegLengthJump1 = 0.12f;//压腿
const float LegLengthJump2 = 0.35f;//蹬腿
const float LegLengthJump3 = 0.18f;//收腿
const float LegLengthJump4 = 0.15f;//落地

const float LegLengthHightFly = 0.25f;//长腿腿长腾空 0.28
const float LegLengthFly = 0.20f;//正常腿长腾空
const float LegLengthHigh2 = 0.30f;//超长腿
const float LegLengthHigh = 0.20f;//长腿 0.23
const float LegLengthNormal = 0.15f;//正常

float x3_balance_zero = 0.00f, x5_balance_zero = 0.00f;//腿摆角角度偏置 机体俯仰角度偏置
float x3_fight_zero = -0.01f;

//位移 速度 yaw wz 左腿摆角 左腿摆角速度 右腿摆角 右腿摆角速度 机体倾角 机体倾角速度 
//左轮转矩 右轮转矩 左腿转矩 右腿转矩
const float K_Array_Wheel[4][10] = //锁腿时轮对旋转的极性要反
{{-2.21772, -3.40667, -4.29163, -3.69989, -19.0027, -3.03794, 4.25754, 0.444385, -2.87482, -1.12261}, 
{-2.21772, -3.40667, 4.29163, 3.69989, 4.25754, 0.444385, -19.0027, -3.03794, -2.87482, -1.12261}, 
{0.0903863, 0.126934, -4.8123, -1.62967, 2.98945, 0.44624, -2.24419, -0.329072, -5.35454, -1.72671}, 
{0.0903863, 0.126934, 4.8123, 1.62967, -2.24419, -0.329072, 2.98945, 0.44624, -5.35454, -1.72671}};

const float K_Array_Fly[4][10] = 
{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 7.42116, 1.11199, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 7.42116, 1.11199, 0, 0}};

const float K_Array_Prone[4][10] =
{{-2.21772, -3.40667, -4.29163, -3.69989, 0, 0, 0, 0, -2.87482, -1.12261}, 
{-2.21772, -3.40667, 4.29163, 3.69989, 0, 0, 0, 0, -2.87482, -1.12261}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

float K_Array_Leg[4][10] = 
{{-1.55655, -3.14219, -1.99572, -0.138159, -13.4528, -2.31259, 
  1.30728, -0.0103002, -3.18659, -1.21733}, {-1.55655, -3.14219, 
  1.99572, 0.138159, 
  1.30728, -0.0103002, -13.4528, -2.31259, -3.18659, -1.21733}, \
{0.124211, 0.237821, -1.78981, -0.700304, 5.75135, 
  0.867569, -4.84826, -0.705913, -6.6076, -2.05861}, {0.124211, 
  0.237821, 1.78981, 0.700304, -4.84826, -0.705913, 5.75135, 
  0.867569, -6.6076, -2.05861}};

const float K_Fit_Array[40][3][3] = 
{{{-1.50521,-0.297581,1.7195},{-0.0947795,-3.46691,0},{2.25187,0,0}},{{-3.2374,-0.604944,3.68256},{1.69947,-8.36906,0},{1.93118,0,0}},{{-4.58451,-3.29834,-1.74883},{23.0411,20.8055,0},{-35.7573,0,0}},{{-1.01722,-1.09305,-0.307284},{7.78084,6.75681,0},{-12.0255,0,0}},{{-2.59861,7.24318,14.3029},{-82.5389,-96.4323,0},{102.31,0,0}},{{-1.25193,0.939706,2.23029},{-7.63048,-13.7727,0},{9.3276,0,0}},{{-5.85245,-12.9176,-9.8191},{59.8311,115.263,0},{-100.409,0,0}},{{-0.915861,-3.25181,1.35827},{10.0255,7.43376,0},{-13.7556,0,0}},{{-5.37686,1.73417,4.04648},{16.2586,-26.6005,0},{0.733271,0,0}},{{-2.09713,0.459176,2.03608},{6.87844,-11.1313,0},{-0.429078,0,0}},{{-1.50521,-0.0947795,2.25187},{-0.297581,-3.46691,0},{1.7195,0,0}},{{-3.2374,1.69947,1.93118},{-0.604944,-8.36906,0},{3.68256,0,0}},{{4.58451,-23.0411,35.7573},{3.29834,-20.8055,0},{1.74883,0,0}},{{1.01722,-7.78084,12.0255},{1.09305,-6.75681,0},{0.307284,0,0}},{{-5.85245,59.8311,-100.409},{-12.9176,115.263,0},{-9.8191,0,0}},{{-0.915861,10.0255,-13.7556},{-3.25181,7.43376,0},{1.35827,0,0}},{{-2.59861,-82.5389,102.31},{7.24318,-96.4323,0},{14.3029,0,0}},{{-1.25193,-7.63048,9.3276},{0.939706,-13.7727,0},{2.23029,0,0}},{{-5.37686,16.2586,0.733271},{1.73417,-26.6005,0},{4.04648,0,0}},{{-2.09713,6.87844,-0.429078},{0.459176,-11.1313,0},{2.03608,0,0}},{{0.204227,2.17237,-3.21514},{-2.80685,0.0815455,0},{3.79536,0,0}},{{0.405015,4.81512,-7.17406},{-6.16084,0.0791303,0},{8.62444,0,0}},{{-1.02566,-2.9701,5.5339},{-3.85695,-2.69276,0},{9.12493,0,0}},{{-0.519274,-0.724588,1.25477},{-0.91573,-0.606744,0},{2.35994,0,0}},{{1.73541,14.4482,-26.1614},{18.3295,20.8337,0},{-36.561,0,0}},{{0.673793,2.56337,-4.27995},{-1.05198,2.3171,0},{0.254012,0,0}},{{-0.732098,-14.4519,21.7411},{-18.8895,-19.9669,0},{39.3244,0,0}},{{-0.431308,1.01953,-1.05428},{-3.1787,-2.72756,0},{6.2188,0,0}},{{-6.39342,7.31168,-7.68526},{-9.14781,-0.721045,0},{11.1429,0,0}},{{-1.96564,3.36939,-3.85301},{-4.16744,-0.301193,0},{5.35293,0,0}},{{0.204227,-2.80685,3.79536},{2.17237,0.0815455,0},{-3.21514,0,0}},{{0.405015,-6.16084,8.62444},{4.81512,0.0791303,0},{-7.17406,0,0}},{{1.02566,3.85695,-9.12493},{2.9701,2.69276,0},{-5.5339,0,0}},{{0.519274,0.91573,-2.35994},{0.724588,0.606744,0},{-1.25477,0,0}},{{-0.732098,-18.8895,39.3244},{-14.4519,-19.9669,0},{21.7411,0,0}},{{-0.431308,-3.1787,6.2188},{1.01953,-2.72756,0},{-1.05428,0,0}},{{1.73541,18.3295,-36.561},{14.4482,20.8337,0},{-26.1614,0,0}},{{0.673793,-1.05198,0.254012},{2.56337,2.3171,0},{-4.27995,0,0}},{{-6.39342,-9.14781,11.1429},{7.31168,-0.721045,0},{-7.68526,0,0}},{{-1.96564,-4.16744,5.35293},{3.36939,-0.301193,0},{-3.85301,0,0}}};

float P_Array[2][8] = 
{{15.3846, -3.86154, -0.0345178, -0.0988772, 0.128298, -0.0130609, -0.0047233, -0.01353}, 
{15.3846, 3.86154, -0.0988772, -0.0345178, -0.0130609, 0.128298, -0.01353, -0.0047233}};
    
const float P_Fit_Array[16][3][3] = 
{{{15.3846,0,0},{0,0,0},{0,0,0}},{{-3.86154,0,0},{0,0,0},{0,0,0}},{{0.0133423,0.00586344,-0.000413327},{-0.316935,-0.00202615,0},{-0.025402,0,0}},{{0.050454,-0.990208,-0.0361836},{0.00192015,-0.000677019,0},{-0.00012967,0,0}},{{0.106136,0.0767992,-0.00588455},{0.0736214,0.00376196,0},{-0.0421193,0,0}},{{-0.0508608,0.241513,-0.155217},{0.0251495,0.00104189,0},{-0.00184647,0,0}},{{-0.00132327,0.00125407,-0.0000458756},{-0.0299709,-0.00317036,0},{0.0495896,0,0}},{{-0.00165051,-0.1033,0.169188},{0.000410662,-0.00103957,0},{-0.0000143599,0,0}},{{15.3846,0,0},{0,0,0},{0,0,0}},{{3.86154,0,0},{0,0,0},{0,0,0}},{{0.050454,0.00192015,-0.00012967},{-0.990208,-0.000677019,0},{-0.0361836,0,0}},{{0.0133423,-0.316935,-0.025402},{0.00586344,-0.00202615,0},{-0.000413327,0,0}},{{-0.0508608,0.0251495,-0.00184647},{0.241513,0.00104189,0},{-0.155217,0,0}},{{0.106136,0.0736214,-0.0421193},{0.0767992,0.00376196,0},{-0.00588455,0,0}},{{-0.00165051,0.000410662,-0.0000143599},{-0.1033,-0.00103957,0},{0.169188,0,0}},{{-0.00132327,-0.0299709,0.0495896},{0.00125407,-0.00317036,0},{-0.0000458756,0,0}}};

wlr_t wlr;
lqr_t lqr;

kalman_filter_t kal_fn[2];

pid_t pid_leg_length[2];
pid_t pid_leg_length_fast[2];
pid_t pid_roll;

static float wlr_fn_calc(float az, float Fy_fdb, float T0_fdb, float L0[3], float theta[3])
{
//    float Fwy = Fy_fdb * cosf(theta[0]) + T0_fdb * sinf(theta[0]) / L0[0];//轮子受到腿部机构竖直方向的作用力
//    float yw_ddot = az
//                    - L0[2] * cosf(theta[0])
//                    + 2 * L0[1] * theta[1] * sinf(theta[0])
//                    + L0[0] * theta[2] * sinf(theta[0])
//                    + L0[0] * powf(theta[1], 2) * cosf(theta[0]);//轮子竖直方向的加速度
//    return Fwy + mw * GRAVITY + mw * yw_ddot;
    
    float Lb = -0.0430315 + 0.65987 * L0[0];
    float dot2_Lb = L0[2] * Lb / L0[0];
    return Fy_fdb * cosf(theta[0]) + ml * (GRAVITY + az - dot2_Lb * cosf(theta[0]));
}

static void k_array_fit(float K[4][10], float Ll_fdb, float Lr_fdb)
{
    float temp;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 10; j++) {
            temp = 0;
            for(int x = 0; x < 3; x++)
                for(int y = 0; x + y < 3; y++)
                    temp += (K_Fit_Array[i * 10 + j][x][y] * powf(Ll_fdb, x) * powf(Lr_fdb, y));
            K[i][j] = temp;
        }
}

static void p_array_fit(float P[2][8], float Ll_fdb, float Lr_fdb)
{
    float temp;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 8; j++) {
            temp = 0;
            for(int x = 0; x < 3; x++)
                for(int y = 0; x + y < 3; y++)
                    temp += (P_Fit_Array[i * 8 + j][x][y] * powf(Ll_fdb, x) * powf(Lr_fdb, y));
            P[i][j] = temp;
        }
}

static void state_predict(void)
{
    for (int i = 0; i < 2; i++) {
        wlr.side[i].predict_wy = 0;
        wlr.side[i].predict_wy += (P_Array[i][0] * lqr.X_fdb[1] + P_Array[i][1] * lqr.X_fdb[3] + P_Array[i][2] * lqr.X_fdb[4] + P_Array[i][3] * lqr.X_fdb[6]);
        for (int j = 0; j < 4; j++) {
            wlr.side[i].predict_wy += (P_Array[i][j + 4] * lqr.U_ref[j]);
        }
    }
}

void wlr_init(void)
{
	wlr.high_set = LegLengthNormal;
	wlr.K_adapt = 0.1f;
    
	twm_init(&twm, BodyWidth, WheelRadius);
	tlm_init(&tlm, LegLengthMax, LegLengthMin, BodyWidth);
	for(int i = 0; i < 2; i++)
	{
		//腿部长度初始化
		vmc_init(&vmc[i], LegLengthParam);
		//卡尔曼滤波器初始化
        kalman_filter_init(&kal_fn[i], 1, 0, 1);
        kal_fn[i].A_data[0] = 1;
        kal_fn[i].H_data[0] = 1;
        kal_fn[i].Q_data[0] = 1;
        kal_fn[i].R_data[0] = 100;
		//PID参数初始化
        pid_init(&pid_leg_length[i], NONE, 500, 0.0f, 10000, 25, 40);//i 2.5f
        pid_init(&pid_leg_length_fast[i], NONE, 1000, 0, 10000, 0, 50);
	}
	//卡尔曼滤波器初始化

	//PID参数初始化
	pid_init(&pid_roll, NONE, 500, 0, 3000, 0, 30);//与VMC的腿长控制协同  1000 0 3500
}

void wlr_protest(void)
{
	pid_leg_length[0].i_out = 0;
	pid_leg_length[1].i_out = 0;
    wlr.s_ref = wlr.s_fdb;
}

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void wlr_control(void)
{
    //------------------------反馈数据更新------------------------//
    wlr.s_fdb = -(wlr.side[0].qy * WheelRadius + wlr.side[1].qy * WheelRadius)/2.0f;
    wlr.v_fdb = -(wlr.side[0].wy * WheelRadius + wlr.side[1].wy * WheelRadius)/2.0f;
    
    if (fabs(wlr.v_fdb) > fabs(wlr.v_ref))//加强超速控制
        wlr.v_ref = data_fusion(wlr.v_ref, 0, fabs(wlr.v_fdb - wlr.v_ref)/2);
    //两侧轮腿分别更新数据
	for(int i = 0; i < 2; i++) {
		//更新腿部VMC模型
		vmc_forward_solution(&vmc[i], wlr.side[i].q1, wlr.side[i].q4, wlr.side[i].w1, \
									  wlr.side[i].w4, wlr.side[i].t1, wlr.side[i].t4);
        //更新预测补偿力矩 用上一个时刻所预测的状态来补偿
        wlr.side[i].T_adapt = wlr.K_adapt * (wlr.side[i].predict_wy + wlr.side[i].wy);
    }
    lqr.X_fdb[0] = wlr.s_fdb;
    lqr.X_fdb[1] = wlr.v_fdb;
    lqr.X_fdb[2] = -wlr.yaw_fdb;
    lqr.X_fdb[3] = -wlr.wz_fdb;
    //机体
    lqr.X_fdb[8] = x5_balance_zero + wlr.pit_fdb;
    lqr.X_fdb[9] = wlr.wy_fdb;
    //左腿
    lqr.X_fdb[4] = x3_balance_zero + (PI / 2 + lqr.X_fdb[8] - vmc[0].q_fdb[0]);
    lqr.X_fdb[5] = lqr.X_fdb[9] - vmc[0].V_fdb.e.w0_fdb;
    lqr.dot_leg_w[0] = (lqr.X_fdb[5] - lqr.last_leg_w[0]) / 0.002f;
    lqr.last_leg_w[0] = lqr.X_fdb[5];
    //右腿
    lqr.X_fdb[6] = x3_balance_zero + (PI / 2 + lqr.X_fdb[8] - vmc[1].q_fdb[0]);
    lqr.X_fdb[7] = lqr.X_fdb[9] - vmc[1].V_fdb.e.w0_fdb;
    lqr.dot_leg_w[1] = (lqr.X_fdb[7] - lqr.last_leg_w[1]) / 0.002f;
    lqr.last_leg_w[1] = lqr.X_fdb[7];
    //支持力解算
    for(int i = 0; i < 2; i++) {
		float L0_array[3] = {vmc[i].L_fdb, vmc[i].V_fdb.e.vy0_fdb, vmc[i].Acc_fdb.L0_ddot};
		float theta_array[3] = {lqr.X_fdb[4+2*i], lqr.X_fdb[5+2*i], lqr.dot_leg_w[i]};
		wlr.side[i].Fn_fdb = wlr_fn_calc(wlr.az_fdb, vmc[i].F_fdb.e.Fy_fdb, vmc[i].F_fdb.e.T0_fdb, L0_array, theta_array);
        kal_fn[i].measured_vector[0] = wlr.side[i].Fn_fdb;
        kalman_filter_update(&kal_fn[i]);
        wlr.side[i].Fn_kal = kal_fn[i].filter_vector[0];
		//离地检测
        if (wlr.high_flag != 2) {
            if(wlr.side[i].Fn_kal < 20.0f)
                wlr.side[i].fly_cnt++;
            else if(wlr.side[i].fly_cnt > 0)
                wlr.side[i].fly_cnt-=2;
            if(wlr.side[i].fly_cnt > 30) {
                wlr.side[i].fly_cnt = 30;
                wlr.side[i].fly_flag = 1;
            } else if(wlr.side[i].fly_cnt == 0)
                wlr.side[i].fly_flag = 0;
        } else {
            wlr.side[i].fly_flag = 0;
            wlr.side[i].fly_cnt = 0;
        }
    }
	//高度选择
	if (wlr.high_flag == 2) {
        wlr.high_set = LegLengthHigh2;
        pid_leg_length[0].kp = 800;
        pid_leg_length[1].kp = 800;
        pid_leg_length[0].kd = 30000;
        pid_leg_length[1].kd = 30000;
    } else if (wlr.high_flag == 1) { //长腿长
        pid_leg_length[0].kp = 500;
        pid_leg_length[1].kp = 500;
        pid_leg_length[0].kd = 10000;
        pid_leg_length[1].kd = 10000;
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
            wlr.high_set = LegLengthHightFly;
        } else {
            wlr.high_set = LegLengthHigh;
        }
    } else { //正常腿长
        pid_leg_length[0].kp = 500;
        pid_leg_length[1].kp = 500;
        pid_leg_length[0].kd = 10000;
        pid_leg_length[1].kd = 10000;
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
            wlr.high_set = LegLengthFly;
        } else {
            wlr.high_set = LegLengthNormal;
        }
    }
	//更新两腿模型
	tlm_gnd_roll_calc(&tlm, -wlr.roll_fdb, vmc[0].L_fdb, vmc[1].L_fdb);//计算地形倾角
    if (wlr.jump_flag != 0 || (wlr.side[0].fly_flag && wlr.side[1].fly_flag))
		tlm.l_ref[0] = tlm.l_ref[1] = wlr.high_set;
	else
        tlm_leg_length_cacl(&tlm, wlr.high_set, 0);//计算腿长设定值
	//------------------------状态选择------------------------//
	//根据当前状态选择合适的控制矩阵
    if (wlr.ctrl_mode == 2) {//力控
        if (wlr.prone_flag) {
            aMartix_Cover(lqr.K, (float*)K_Array_Prone, 4, 10);
        } else if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
            aMartix_Cover(lqr.K, (float*)K_Array_Fly, 4, 10);
        } else {
            k_array_fit(K_Array_Leg, vmc[0].L_fdb, vmc[1].L_fdb);
            aMartix_Cover(lqr.K, (float*)K_Array_Leg, 4, 10);
        }
    } else if (wlr.ctrl_mode == 1) {//位控
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
            aMartix_Cover(lqr.K, (float*)K_Array_Fly, 4, 10);
        } else {
            aMartix_Cover(lqr.K, (float*)K_Array_Wheel, 4, 10);
        }
    }
	//------------------------控制数据更新------------------------//
	//全身运动控制
	wlr.roll_offs = pid_calc(&pid_roll, 0, wlr.roll_fdb);
    wlr.inertial_offs = (mb/2) * wlr.high_set * lqr.X_fdb[3] * lqr.X_fdb[1] / (BodyWidth/2) / 2;//惯性力补偿
    lqr.X_ref[0] = wlr.s_ref;
//    data_limit(&lqr.X_ref[0], lqr.X_fdb[0]-1.0f, lqr.X_fdb[0]+1.0f);
    lqr.X_ref[1] = wlr.v_ref;
    lqr.X_ref[2] = -wlr.yaw_ref;
    lqr.X_ref[3] = -wlr.wz_ref;
    //期望限制
    wlr.K_ref = 0;
    for (int i = 0; i < 2; i++) {
        float K_temp;
        if (wlr.side[i].q1 > 3.6f)
            K_temp = (wlr.side[i].q1 - 3.6f)/0.2f;
        if (K_temp > wlr.K_ref)
            wlr.K_ref = K_temp;
        if (wlr.side[i].q4 < -0.5f)
            K_temp = (-0.5f - wlr.side[i].q4)/0.2f;
        if (K_temp > wlr.K_ref)
            wlr.K_ref = K_temp;
    }
    for (int i = 0; i < 4; i++) {
        lqr.X_ref[i] = data_fusion(lqr.X_ref[i], lqr.X_fdb[i], wlr.K_ref);
    }
    aMartix_Add(1, lqr.X_ref, -1, lqr.X_fdb, lqr.X_diff, 10, 1);
    aMartix_Mul(lqr.K, lqr.X_diff, lqr.U_ref, 4, 10, 1);
    //预测下一个时刻的状态
    p_array_fit(P_Array, vmc[0].L_fdb, vmc[1].L_fdb);
    state_predict();
	//虚拟力映射
	for (int i = 0; i < 2; i++) {
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {            //浮空收腿 响应不用那么大
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		} else																//常态 跳跃压腿阶段 跳跃落地阶段
            wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
                                 + mb*GRAVITY/2 + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);

		wlr.side[i].T0 = -lqr.U_ref[2+i];
		vmc_inverse_solution(&vmc[i], wlr.high_set, PI / 2 + x3_balance_zero, wlr.side[i].T0, wlr.side[i].Fy);
	}
	//------------------------控制数据输出------------------------//
	for (int i = 0; i < 2; i++) {
		wlr.side[i].T1 =  vmc[i].T_ref.e.T1_ref;
		wlr.side[i].T4 =  vmc[i].T_ref.e.T4_ref;
        wlr.side[i].Tw = -lqr.U_ref[i] - wlr.side[i].T_adapt;
		wlr.side[i].P1 =  vmc[i].q_ref[1];
		wlr.side[i].P4 =  vmc[i].q_ref[4];
	}
}
