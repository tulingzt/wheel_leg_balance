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

//上台阶参数设定
float jump_vset  = 2.0f;
float jump_theta = 0.0f;
float jump_pitch = 0.26f;
float jump_highset1 = 0.30f;
float jump_highset2 = 0.17f;
float jump_highset3 = 0.20f;

float x3_balance_zero = 0.00f, x5_balance_zero = 0.00f;//腿摆角角度偏置 机体俯仰角度偏置
float x3_fight_zero = -0.01f;

//位移 速度 yaw wz 左腿摆角 左腿摆角速度 右腿摆角 右腿摆角速度 机体倾角 机体倾角速度 
//左轮转矩 右轮转矩 左腿转矩 右腿转矩
const float K_Array_Wheel[4][10] =
{{-1.57274, -3.15546, -1.2459, -0.144771, -21.6562, -3.41203, 6.88813,
   0.587245, -2.08519, -0.764707}, {-1.57274, -3.15546, 1.2459, 
  0.144771, 6.88813, 
  0.587245, -21.6562, -3.41203, -2.08519, -0.764707}, {0.0727682, 
  0.137973, -1.47971, -0.727712, 9.58579, 
  1.29604, -8.82474, -1.18285, -7.44177, -2.14868}, {0.0727682, 
  0.137973, 1.47971, 0.727712, -8.82474, -1.18285, 9.58579, 
  1.29604, -7.44177, -2.14868}};

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
{{{-1.51693,0.666829,0.0020757},{-0.973766,-2.19271,0},{2.56893,0,0}},{{-3.35851,1.69293,-0.26793},{-0.230284,-5.90172,0},{2.95703,0,0}},{{-4.62683,-4.9614,2.42412},{14.9218,15.0609,0},{-21.0028,0,0}},{{-1.473,-1.92018,0.914779},{6.18619,6.43388,0},{-8.68405,0,0}},{{-3.99236,16.2818,-1.35781},{-66.6393,-87.4071,0},{69.6387,0,0}},{{-1.50839,2.56442,-0.325681},{-6.51526,-12.8993,0},{6.085,0,0}},{{-5.22371,-22.1247,3.73534},{43.1643,112.403,0},{-69.2897,0,0}},{{-1.00345,-3.69224,2.14492},{7.69622,7.89866,0},{-9.73917,0,0}},{{-5.84992,5.12534,0.797933},{13.7652,-26.1314,0},{3.09949,0,0}},{{-2.3416,2.09194,0.289239},{5.81479,-11.1119,0},{0.879299,0,0}},{{-1.51693,-0.973766,2.56893},{0.666829,-2.19271,0},{0.0020757,0,0}},{{-3.35851,-0.230284,2.95703},{1.69293,-5.90172,0},{-0.26793,0,0}},{{4.62683,-14.9218,21.0028},{4.9614,-15.0609,0},{-2.42412,0,0}},{{1.473,-6.18619,8.68405},{1.92018,-6.43388,0},{-0.914779,0,0}},{{-5.22371,43.1643,-69.2897},{-22.1247,112.403,0},{3.73534,0,0}},{{-1.00345,7.69622,-9.73917},{-3.69224,7.89866,0},{2.14492,0,0}},{{-3.99236,-66.6393,69.6387},{16.2818,-87.4071,0},{-1.35781,0,0}},{{-1.50839,-6.51526,6.085},{2.56442,-12.8993,0},{-0.325681,0,0}},{{-5.84992,13.7652,3.09949},{5.12534,-26.1314,0},{0.797933,0,0}},{{-2.3416,5.81479,0.879299},{2.09194,-11.1119,0},{0.289239,0,0}},{{0.221878,1.68607,-2.40537},{-2.33664,0.263191,0},{2.78542,0,0}},{{0.454467,3.94566,-5.69408},{-5.396,0.46841,0},{6.83071,0,0}},{{-0.937333,-2.84647,5.41048},{-3.76335,-5.67824,0},{8.68558,0,0}},{{-0.531198,-1.02382,1.84102},{-1.33007,-2.19337,0},{3.14589,0,0}},{{1.49091,16.0069,-31.5836},{22.9241,46.8607,0},{-44.3842,0,0}},{{0.722837,2.93499,-5.02984},{-0.779144,5.23333,0},{-0.380675,0,0}},{{-0.290652,-18.1231,27.4889},{-21.67,-45.5711,0},{46.4295,0,0}},{{-0.406269,0.735876,-0.612306},{-3.74982,-5.47615,0},{7.22029,0,0}},{{-7.90231,7.5331,-6.27229},{-9.66904,-0.368691,0},{9.77188,0,0}},{{-2.54615,3.48879,-3.2685},{-4.4234,-0.139044,0},{4.79118,0,0}},{{0.221878,-2.33664,2.78542},{1.68607,0.263191,0},{-2.40537,0,0}},{{0.454467,-5.396,6.83071},{3.94566,0.46841,0},{-5.69408,0,0}},{{0.937333,3.76335,-8.68558},{2.84647,5.67824,0},{-5.41048,0,0}},{{0.531198,1.33007,-3.14589},{1.02382,2.19337,0},{-1.84102,0,0}},{{-0.290652,-21.67,46.4295},{-18.1231,-45.5711,0},{27.4889,0,0}},{{-0.406269,-3.74982,7.22029},{0.735876,-5.47615,0},{-0.612306,0,0}},{{1.49091,22.9241,-44.3842},{16.0069,46.8607,0},{-31.5836,0,0}},{{0.722837,-0.779144,-0.380675},{2.93499,5.23333,0},{-5.02984,0,0}},{{-7.90231,-9.66904,9.77188},{7.5331,-0.368691,0},{-6.27229,0,0}},{{-2.54615,-4.4234,4.79118},{3.48879,-0.139044,0},{-3.2685,0,0}}};

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
    float Fwy = Fy_fdb * cosf(theta[0]) + T0_fdb * sinf(theta[0]) / L0[0];//轮子受到腿部机构竖直方向的作用力
    float yw_ddot = az
                    - L0[2] * cosf(theta[0])
                    + 2 * L0[1] * theta[1] * sinf(theta[0])
                    + L0[0] * theta[2] * sinf(theta[0])
                    + L0[0] * powf(theta[1], 2) * cosf(theta[0]);//轮子竖直方向的加速度
    return Fwy + mw * GRAVITY + mw * yw_ddot;
    
//    float Lb = -0.0430315 + 0.65987 * L0[0];
//    float dot2_Lb = L0[2] * Lb / L0[0];
//    return Fy_fdb * cosf(theta[0]) + ml * (GRAVITY + az - dot2_Lb * cosf(theta[0]));
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
        pid_init(&pid_leg_length[i], NONE, 300, 0.0f, 60000, 25, 40);//500 0/2.5f 10000
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
	wlr.s_adapt = wlr.s_fdb;
}

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void wlr_control(void)
{
    //------------------------反馈数据更新------------------------//
    wlr.s_fdb = -(wlr.side[0].qy * WheelRadius + wlr.side[1].qy * WheelRadius)/2.0f;
    wlr.v_fdb = -(wlr.side[0].wy * WheelRadius + wlr.side[1].wy * WheelRadius)/2.0f;
    
    if (fabs(wlr.v_fdb) > fabs(wlr.v_ref))//加强超速控制
        wlr.v_ref = data_fusion(wlr.v_ref, 0, fabs(wlr.v_fdb - wlr.v_ref));
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
        pid_leg_length[0].kp = pid_leg_length[1].kp = 800;
        pid_leg_length[0].kd = pid_leg_length[1].kd = 30000;
    } else if (wlr.high_flag == 1) { //上坡腿长
        pid_leg_length[0].kp = pid_leg_length[1].kp = 500;
        pid_leg_length[0].kd = pid_leg_length[1].kd = 10000;
		data_limit(&wlr.v_ref,-1.2f,1.2f);
		data_limit(&wlr.v_ref,wlr.v_fdb-0.7f,wlr.v_fdb+0.7f);
		if (fabs(wlr.v_fdb) > fabs(wlr.v_ref))//加强超速控制
			wlr.v_ref = data_fusion(wlr.v_ref, 0, fabs(wlr.v_fdb - wlr.v_ref)/0.9);
        wlr.high_set = LegLengthHigh;
    } else { //正常腿长
        pid_leg_length[0].kp = pid_leg_length[1].kp = 500;
        wlr.high_set = LegLengthNormal;
    }
    //上台阶
    for (int i = 0; i < 2; i++) {
        if (wlr.jump_flag == 1) {
            wlr.high_set = jump_highset1;
            wlr.v_ref = jump_vset;
            lqr.X_ref[4] = lqr.X_ref[6] = jump_theta;
            if (fabs(chassis_imu.pit) > jump_pitch){
                wlr.jump_flag = 2;
                lqr.X_ref[4]  = 0;
                lqr.X_ref[6]  = 0;
            }
        } else if (wlr.jump_flag == 2) {
            wlr.high_set = jump_highset2;
            if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
                wlr.jump_flag = 3;
        } else if (wlr.jump_flag == 3) {
            wlr.high_set = jump_highset3;
            wlr.jump_cnt++;
            if (wlr.jump_cnt > 200) {
                wlr.jump_cnt = 0;
                wlr.jump_flag = 4;
            }				
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
//        } else if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
//            aMartix_Cover(lqr.K, (float*)K_Array_Fly, 4, 10);
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
    if (wlr.v_ref == 0 && wlr.jump_flag == 0)
    {
        lqr.X_ref[0] = wlr.s_adapt;
    } else {
        lqr.X_ref[0] = wlr.s_fdb;
        wlr.s_adapt = wlr.s_fdb;
    }
    lqr.X_ref[1] = wlr.v_ref;
    lqr.X_ref[2] = -wlr.yaw_ref;
    lqr.X_ref[3] = -wlr.wz_ref;
    //期望限制
    wlr.K_ref = 0;
    for (int i = 0; i < 2; i++) {
        float K_temp;
        if (wlr.side[i].q1 > 3.6f)//电机初始位置改变
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
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag && wlr.jump_flag==0) {            //浮空收腿 响应不用那么大
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		} else if (wlr.jump_flag == 2){
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		} else if (wlr.jump_flag == 3){
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + mb*GRAVITY/2 + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);			
		} else														                       //常态 跳跃压腿阶段 跳跃落地阶段
            wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
                                 + mb*GRAVITY/2 + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);

		wlr.side[i].T0 = -lqr.U_ref[2+i];
		vmc_inverse_solution(&vmc[i], wlr.high_set, PI / 2 + x3_balance_zero, wlr.side[i].T0, wlr.side[i].Fy);
	}
    //------------------------控制数据输出------------------------//
    for (int i = 0; i < 2; i++) {
        wlr.side[i].T1 =  vmc[i].T_ref.e.T1_ref;
        wlr.side[i].T4 =  vmc[i].T_ref.e.T4_ref;
        wlr.side[i].Tw = -lqr.U_ref[i] ;//- wlr.side[i].T_adapt;
        wlr.side[i].P1 =  vmc[i].q_ref[1];
        wlr.side[i].P4 =  vmc[i].q_ref[4];
    }
}
