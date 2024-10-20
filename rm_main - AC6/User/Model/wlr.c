#include "wlr.h"
#include "chassis_task.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "prot_imu.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "prot_dr16.h"
#include "pid.h"
#include "kalman_filter.h"
#include "math_lib.h"
#include "math_matrix.h"
#include "KNN.h"

#define WLR_SIGN(x) ((x) > 0? (1): (-1))

#define CHASSIS_PERIOD_DU 2

const float LegLengthParam[5] = {0.150f, 0.270f, 0.270f, 0.150f, 0.150f};
float mb = 7.75f;
//float mb = 4.4f;//原先值
float ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.51f;//两轮间距
//const float WheelRadius = 0.065f;//轮子半径 小轮子
const float WheelRadius = 0.1f;//轮子半径 气胎
const float LegLengthMax = 0.35f, LegLengthMin = 0.15f;

const float LegLengthJump1 = 0.12f;//压腿
const float LegLengthJump2 = 0.35f;//蹬腿
const float LegLengthJump3 = 0.18f;//收腿
const float LegLengthJump4 = 0.15f;//落地

const float LegLengthHighFly = 0.25f;//长腿腿长腾空 0.28
const float LegLengthFly = 0.20f;//正常腿长腾空
const float LegLengthHigh2 = 0.30f;//超长腿
const float LegLengthHigh = 0.20f;//长腿 0.23
const float LegLengthNormal = 0.15f;//正常

//上台阶参数设定
float jump_vset  = 2.0f;
float jump_theta = 0.0f;
float jump_pitch = 0.25f;
float jump_highset1 = 0.30f;
float jump_highset2 = 0.17f;
float jump_highset3 = 0.12f;

float x3_balance_zero = 0.00f, x5_balance_zero = 0.00f;//腿摆角角度偏置 机体俯仰角度偏置
float x3_fight_zero = -0.00f;

uint8_t state;

//位移 速度 yaw wz 左腿摆角 左腿摆角速度 右腿摆角 右腿摆角速度 机体倾角 机体倾角速度 
//左轮转矩 右轮转矩 左腿转矩 右腿转矩
const float K_Array_Wheel[4][10] =
{{-1.57274, -3.15546, -1.2459, -0.144771, -21.6562, -3.41203, 6.88813, 0.587245, -2.08519, -0.764707}, 
{-1.57274, -3.15546, 1.2459, 0.144771, 6.88813, 0.587245, -21.6562, -3.41203, -2.08519, -0.764707}, 
{0.0727682, 
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
{{10, 10, -10, -5, 0, 0, 0, 0, 0, 0}, 
{10, 10, 10, 5, 0, 0, 0, 0, 0, 0}, 
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
  
// 0731_1: 3000, 3000, 5000, 200, 20000, 500, 20000, 500, 20000, 800
//float K_Array_Leg_020[4][10] = 
//{{-2.78922, -4.85939, -3.58771, -0.91893, -13.3172, -2.36485, -7.5601, -1.39973, -4.76936, -1.55292}, 
//{-2.78922, -4.85939, 3.58771, 0.91893, -7.5601, -1.39973, -13.3172, -2.36485, -4.76936, -1.55292}, 
//{4.0807, 6.4926, -5.33503, -1.52053, 30.8647, 4.97539, -7.7491, -0.730829, -19.1712, -3.94271}, 
//{4.0807, 6.4926, 5.33503, 1.52053, -7.7491, -0.730829, 30.8647, 4.97539, -19.1712, -3.94271}};

// 0701_1: 1000, 3000, 5000, 200, 20000, 500, 20000, 500, 20000, 800
//float K_Array_Leg_020[4][10] = 
//{{-1.6259, -4.07334, -3.58771, -0.91893, -12.7925, -2.24798, \
//-7.03533, -1.28285, -4.59005, -1.46792}, {-1.6259, -4.07334, 3.58771, 
//  0.91893, -7.03533, -1.28285, -12.7925, -2.24798, -4.59005, \
//-1.46792}, {2.27449, 5.37824, -5.33503, -1.52053, 30.1202, 
//  4.8103, -8.49354, -0.89592, -19.4272, -4.06245}, {2.27449, 5.37824, 
//  5.33503, 1.52053, -8.49354, -0.89592, 30.1202, 
//  4.8103, -19.4272, -4.06245}};

// 0701_2: 1000, 1500, 5000, 200, 20000, 500, 20000, 500, 20000, 800
//float K_Array_Leg_020[4][10] = 
//{{-1.64807, -3.50205, -3.58771, -0.91893, -12.2241, -2.11847, -6.467, \
//-1.15335, -4.33904, -1.36173}, {-1.64807, -3.50205, 3.58771, 
//  0.91893, -6.467, -1.15335, -12.2241, -2.11847, -4.33904, -1.36173}, \
//{2.15151, 4.29076, -5.33503, -1.52053, 29.2192, 
//  4.61783, -9.39454, -1.08839, -19.7782, -4.20321}, {2.15151, 4.29076,
//   5.33503, 1.52053, -9.39454, -1.08839, 29.2192, 
//  4.61783, -19.7782, -4.20321}};

//// 0801_3: 2000, 4000, 5000, 200, 20000, 500, 20000, 500, 20000, 800

//位移 速度 yaw wz 左腿摆角 左腿摆角速度 右腿摆角 右腿摆角速度 机体倾角 机体倾角速度 
//左轮转矩 右轮转矩 左腿转矩 右腿转矩
//float K_Array_Leg_020[4][10] = 
//{{-2.78922, -4.85939, -3.58771, -0.91893, -13.3172, -2.36485, -7.5601, -1.39973, -4.76936, -1.55292}, 
//{-2.78922, -4.85939, 3.58771, 0.91893, -7.5601, -1.39973, -13.3172, -2.36485, -4.76936, -1.55292},
//{4.0807, 6.4926, -5.33503, -1.52053, 30.8647, 4.97539, -7.7491, -0.730829, -19.1712, -3.94271},
//{4.0807, 6.4926, 5.33503, 1.52053, -7.7491, -0.730829, 30.8647, 4.97539, -19.1712, -3.94271}};


// float K_Array_Leg_020[4][10] =
//     {{-1.51845, -3.27599, -2.63092, -0.72509, -9.57717, -1.69473, \
//-6.06595, -1.08269, -3.89725, -1.2591}, {-1.51845, -3.27599, 2.63092, 
//  0.72509, -6.06595, -1.08269, -9.57717, -1.69473, -3.89725, \
//-1.2591}, {3.92616, 7.87021, -4.60154, -1.36436, 33.9862, 
//  5.54002, -5.70371, -0.338145, -17.8171, -3.52179}, {3.92616, 
//  7.87021, 4.60154, 1.36436, -5.70371, -0.338145, 33.9862, 
//  5.54002, -17.8171, -3.52179}}; //不怕踩

 float K_Array_Leg_020[4][10] =
{{-1.89955, -3.55006, -3.37568, -0.931398, -9.56488, -2.07686, \
-7.36839, -1.38847, -4.47578, -1.74204}, {-1.89955, -3.55006, 3.37568,
   0.931398, -7.36839, -1.38847, -9.56488, -2.07686, -4.47578, \
-1.74204}, {5.27578, 9.25302, -4.70066, -1.36434, 37.0734, 
  9.11907, -4.05081, -1.21996, -25.8654, -8.013}, {5.27578, 9.25302, 
  4.70066, 1.36434, -4.05081, -1.21996, 37.0734, 
  9.11907, -25.8654, -8.013}};

// 0805 : 4000, 2000, 5000, 200, 20000, 500, 20000, 500, 20000, 800
float K_Array_Leg_030[4][10] = 
{{-2.28967, -4.86036, -3.58771, -0.91893, -13.4722, -2.3976, \
-7.71502, -1.43248, -5.55583, -1.60581}, {-2.28967, -4.86036, 3.58771,
   0.91893, -7.71502, -1.43248, -13.4722, -2.3976, -5.55583, \
-1.60581}, {3.26812, 6.49503, -5.33503, -1.52053, 30.802, 
  4.96946, -7.81174, -0.736757, -23.7322, -4.07994}, {3.26812, 
  6.49503, 5.33503, 1.52053, -7.81174, -0.736757, 30.802, 
  4.96946, -23.7322, -4.07994}};

const float K_Fit_Array[40][3][3] = 
{{{-1.50772,-0.201288,1.74873},{-0.174393,-3.76534,0},{2.48358,0,0}},{{-3.24058,-0.347737,3.66648},{1.46602,-9.05607,0},{2.57215,0,0}},{{-4.91297,-4.4151,-0.53714},{22.322,21.6979,0},{-33.8252,0,0}},{{-1.27698,-1.59112,0.0465232},{8.25481,7.83571,0},{-12.5106,0,0}},{{-2.61953,10.6752,11.36},{-81.2722,-103.514,0},{98.6671,0,0}},{{-1.21525,1.47504,1.8075},{-7.82801,-14.9352,0},{9.59973,0,0}},{{-5.83705,-16.7586,-6.51043},{59.0041,123.214,0},{-98.0578,0,0}},{{-0.952356,-3.52993,1.66281},{9.96847,8.02318,0},{-13.3617,0,0}},{{-5.75986,2.8643,3.66412},{16.2791,-28.6718,0},{2.21668,0,0}},{{-2.29619,0.953316,1.8891},{6.9863,-12.3387,0},{0.36472,0,0}},{{-1.50772,-0.174393,2.48358},{-0.201288,-3.76534,0},{1.74873,0,0}},{{-3.24058,1.46602,2.57215},{-0.347737,-9.05607,0},{3.66648,0,0}},{{4.91297,-22.322,33.8252},{4.4151,-21.6979,0},{0.53714,0,0}},{{1.27698,-8.25481,12.5106},{1.59112,-7.83571,0},{-0.0465232,0,0}},{{-5.83705,59.0041,-98.0578},{-16.7586,123.214,0},{-6.51043,0,0}},{{-0.952356,9.96847,-13.3617},{-3.52993,8.02318,0},{1.66281,0,0}},{{-2.61953,-81.2722,98.6671},{10.6752,-103.514,0},{11.36,0,0}},{{-1.21525,-7.82801,9.59973},{1.47504,-14.9352,0},{1.8075,0,0}},{{-5.75986,16.2791,2.21668},{2.8643,-28.6718,0},{3.66412,0,0}},{{-2.29619,6.9863,0.36472},{0.953316,-12.3387,0},{1.8891,0,0}},{{0.199645,2.23515,-3.25561},{-2.85191,0.0883145,0},{3.7967,0,0}},{{0.398063,4.96975,-7.30163},{-6.28643,0.106763,0},{8.6668,0,0}},{{-0.930465,-3.21701,6.30601},{-4.35736,-4.18175,0},{10.1397,0,0}},{{-0.509699,-0.932181,1.7568},{-1.25037,-1.25889,0},{3.07325,0,0}},{{1.56636,15.7418,-29.8362},{20.2154,28.388,0},{-40.6369,0,0}},{{0.663193,2.78914,-4.79139},{-0.884351,3.17267,0},{-0.243134,0,0}},{{-0.56926,-15.7434,25.1009},{-20.7429,-27.7502,0},{43.8098,0,0}},{{-0.422595,0.948891,-0.72993},{-3.4919,-3.54838,0},{6.83277,0,0}},{{-7.12249,7.53699,-7.6356},{-9.45193,-0.49529,0},{10.9732,0,0}},{{-2.32959,3.55112,-3.91312},{-4.39327,-0.217266,0},{5.39055,0,0}},{{0.199645,-2.85191,3.7967},{2.23515,0.0883145,0},{-3.25561,0,0}},{{0.398063,-6.28643,8.6668},{4.96975,0.106763,0},{-7.30163,0,0}},{{0.930465,4.35736,-10.1397},{3.21701,4.18175,0},{-6.30601,0,0}},{{0.509699,1.25037,-3.07325},{0.932181,1.25889,0},{-1.7568,0,0}},{{-0.56926,-20.7429,43.8098},{-15.7434,-27.7502,0},{25.1009,0,0}},{{-0.422595,-3.4919,6.83277},{0.948891,-3.54838,0},{-0.72993,0,0}},{{1.56636,20.2154,-40.6369},{15.7418,28.388,0},{-29.8362,0,0}},{{0.663193,-0.884351,-0.243134},{2.78914,3.17267,0},{-4.79139,0,0}},{{-7.12249,-9.45193,10.9732},{7.53699,-0.49529,0},{-7.6356,0,0}},{{-2.32959,-4.39327,5.39055},{3.55112,-0.217266,0},{-3.91312,0,0}}};

const float K_Fit_Array2[40][3][3] = 
{{{-2.01738,0.619157,0.561419},{-1.14183,-3.81284,0},{3.92777,0,0}},{{-4.169,1.22219,1.23516},{0.0687191,-9.18838,0},{4.45458,0,0}},{{-5.19993,-4.09213,-0.850394},{22.5581,22.4976,0},{-34.3481,0,0}},{{-1.29569,-1.28832,-0.277364},{7.94806,7.84303,0},{-12.0676,0,0}},{{-3.25979,11.1385,9.25514},{-82.7641,-103.138,0},{97.9959,0,0}},{{-1.36892,1.60971,1.30682},{-7.92552,-14.6136,0},{8.51295,0,0}},{{-6.48552,-17.0253,-5.67787},{56.7834,120.048,0},{-94.1846,0,0}},{{-1.1207,-3.21235,0.565515},{9.59504,7.62117,0},{-12.7821,0,0}},{{-6.26313,4.25921,1.11382},{16.4493,-27.7753,0},{1.58917,0,0}},{{-2.55444,1.65792,0.638259},{7.15658,-11.9863,0},{-0.0380284,0,0}},{{-2.01738,-1.14183,3.92777},{0.619157,-3.81284,0},{0.561419,0,0}},{{-4.169,0.0687191,4.45458},{1.22219,-9.18838,0},{1.23516,0,0}},{{5.19993,-22.5581,34.3481},{4.09213,-22.4976,0},{0.850394,0,0}},{{1.29569,-7.94806,12.0676},{1.28832,-7.84303,0},{0.277364,0,0}},{{-6.48552,56.7834,-94.1846},{-17.0253,120.048,0},{-5.67787,0,0}},{{-1.1207,9.59504,-12.7821},{-3.21235,7.62117,0},{0.565515,0,0}},{{-3.25979,-82.7641,97.9959},{11.1385,-103.138,0},{9.25514,0,0}},{{-1.36892,-7.92552,8.51295},{1.60971,-14.6136,0},{1.30682,0,0}},{{-6.26313,16.4493,1.58917},{4.25921,-27.7753,0},{1.11382,0,0}},{{-2.55444,7.15658,-0.0380284},{1.65792,-11.9863,0},{0.638259,0,0}},{{0.285877,2.49044,-3.64987},{-3.36299,0.260743,0},{4.29609,0,0}},{{0.542454,5.41621,-7.98522},{-7.17667,0.391461,0},{9.5801,0,0}},{{-0.949327,-3.46488,6.47959},{-4.60436,-3.96777,0},{10.4229,0,0}},{{-0.510705,-0.982773,1.71583},{-1.26871,-1.07161,0},{2.99343,0,0}},{{1.6327,16.3883,-30.1977},{19.9453,28.1593,0},{-40.0339,0,0}},{{0.682502,2.91383,-4.89648},{-0.996772,3.16041,0},{-0.0729002,0,0}},{{-0.468162,-15.3817,24.1509},{-21.6527,-27.342,0},{44.3754,0,0}},{{-0.404202,1.04296,-0.84007},{-3.67243,-3.5285,0},{6.98738,0,0}},{{-7.0644,7.65992,-7.7833},{-9.85481,-0.385766,0},{11.4317,0,0}},{{-2.30102,3.62527,-4.01723},{-4.60912,-0.162355,0},{5.65669,0,0}},{{0.285877,-3.36299,4.29609},{2.49044,0.260743,0},{-3.64987,0,0}},{{0.542454,-7.17667,9.5801},{5.41621,0.391461,0},{-7.98522,0,0}},{{0.949327,4.60436,-10.4229},{3.46488,3.96777,0},{-6.47959,0,0}},{{0.510705,1.26871,-2.99343},{0.982773,1.07161,0},{-1.71583,0,0}},{{-0.468162,-21.6527,44.3754},{-15.3817,-27.342,0},{24.1509,0,0}},{{-0.404202,-3.67243,6.98738},{1.04296,-3.5285,0},{-0.84007,0,0}},{{1.6327,19.9453,-40.0339},{16.3883,28.1593,0},{-30.1977,0,0}},{{0.682502,-0.996772,-0.0729002},{2.91383,3.16041,0},{-4.89648,0,0}},{{-7.0644,-9.85481,11.4317},{7.65992,-0.385766,0},{-7.7833,0,0}},{{-2.30102,-4.60912,5.65669},{3.62527,-0.162355,0},{-4.01723,0,0}}};

	
float P_Array[2][8] = 
{{15.3846, -3.86154, -0.0345178, -0.0988772, 0.128298, -0.0130609, -0.0047233, -0.01353}, 
{15.3846, 3.86154, -0.0988772, -0.0345178, -0.0130609, 0.128298, -0.01353, -0.0047233}};
    
const float P_Fit_Array[16][3][3] = 
{{{15.3846,0,0},{0,0,0},{0,0,0}},{{-3.86154,0,0},{0,0,0},{0,0,0}},{{0.0133423,0.00586344,-0.000413327},{-0.316935,-0.00202615,0},{-0.025402,0,0}},{{0.050454,-0.990208,-0.0361836},{0.00192015,-0.000677019,0},{-0.00012967,0,0}},{{0.106136,0.0767992,-0.00588455},{0.0736214,0.00376196,0},{-0.0421193,0,0}},{{-0.0508608,0.241513,-0.155217},{0.0251495,0.00104189,0},{-0.00184647,0,0}},{{-0.00132327,0.00125407,-0.0000458756},{-0.0299709,-0.00317036,0},{0.0495896,0,0}},{{-0.00165051,-0.1033,0.169188},{0.000410662,-0.00103957,0},{-0.0000143599,0,0}},{{15.3846,0,0},{0,0,0},{0,0,0}},{{3.86154,0,0},{0,0,0},{0,0,0}},{{0.050454,0.00192015,-0.00012967},{-0.990208,-0.000677019,0},{-0.0361836,0,0}},{{0.0133423,-0.316935,-0.025402},{0.00586344,-0.00202615,0},{-0.000413327,0,0}},{{-0.0508608,0.0251495,-0.00184647},{0.241513,0.00104189,0},{-0.155217,0,0}},{{0.106136,0.0736214,-0.0421193},{0.0767992,0.00376196,0},{-0.00588455,0,0}},{{-0.00165051,0.000410662,-0.0000143599},{-0.1033,-0.00103957,0},{0.169188,0,0}},{{-0.00132327,-0.0299709,0.0495896},{0.00125407,-0.00317036,0},{-0.0000458756,0,0}}};

wlr_t wlr;
lqr_t lqr;

kalman_filter_t kal_fn[2];
    
ramp_t height_ramp;

pid_t pid_leg_length[2];
pid_t pid_leg_length_fast[2];
pid_t pid_leg_vy[2];
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
            wlr.side[i].predict_wy -= (P_Array[i][j + 4] * wlr.side[i].Tw);
        }
    }
}

void wlr_init(void)
{
	wlr.high_set = LegLengthNormal;
	wlr.K_adapt = 0.1f;
    
	twm_init(&twm, BodyWidth, WheelRadius);
	tlm_init(&tlm, LegLengthMax, LegLengthMin, BodyWidth);
    
    ramp_init(&height_ramp, 0.001f, 0.1f, 0.3f);
	for(int i = 0; i < 2; i++) {
		//腿部长度初始化
		vmc_init(&vmc[i], LegLengthParam);
		//卡尔曼滤波器初始化
        kalman_filter_init(&kal_fn[i], 1, 0, 1);
        kal_fn[i].A_data[0] = 1;
        kal_fn[i].H_data[0] = 1;
        kal_fn[i].Q_data[0] = 1;
        kal_fn[i].R_data[0] = 100;
        
		//PID参数初始化      
        pid_init(&pid_leg_length[i], NONE, 1000, 1.0f,  0.0f, 20, 50);//500 0/2.5f 10000
        pid_init(&pid_leg_length_fast[i], NONE, 500, 0,30000, 0, 50);
        pid_init(&pid_leg_vy[i], NONE, 50, 0, 0, 0, 200);
	}
	//卡尔曼滤波器初始化

	//PID参数初始化
	pid_init(&pid_roll, NONE, 300, 0, 5000, 0, 30);//与VMC的腿长控制协同  1000 0 3500
}

void wlr_protest(void)
{
	pid_leg_length[0].i_out = 0;
	pid_leg_length[1].i_out = 0;
    height_ramp.out = 0.1f;
    wlr.s_ref = wlr.s_fdb;
	wlr.s_adapt = wlr.s_fdb;
}

float pid_p = 1000.0f, pid_i = 1.0f, pid_d = 10000.0f;
float pid_p_fly = 3000.0f,pid_d_fly = 80000.0f;

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void wlr_control(void)
{
    //------------------------反馈数据更新------------------------//
    wlr.s_fdb = -(wlr.side[0].qy * WheelRadius + wlr.side[1].qy * WheelRadius)/2.0f;
    wlr.v_fdb = -(wlr.side[0].wy * WheelRadius + wlr.side[1].wy * WheelRadius)/2.0f;
    
//    if (fabs(wlr.v_fdb) > fabs(wlr.v_ref))//加强超速控制
//        wlr.v_ref = data_fusion(wlr.v_ref, 0, fabs(wlr.v_fdb - wlr.v_ref));
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
//		//离地检测
       
        
        if (wlr.high_flag != 2) {
//           if(wlr.side[i].Fn_kal < 10.0f)
           if( knn_classify(input_data, 8))
                wlr.side[i].fly_cnt++;
            else if(wlr.side[i].fly_cnt > 0)
                wlr.side[i].fly_cnt-=2;
            if(wlr.side[i].fly_cnt > 30) {
                wlr.side[i].fly_cnt = 30;
                wlr.side[i].fly_flag = 1;
            } else if(wlr.side[i].fly_cnt == 0)
                wlr.side[i].fly_flag = 0;
            if(wlr.side[i].fly_flag)
           wlr.side[i].fly_adapt = 1;//累计腾空250ms
            else if(!wlr.side[i].fly_flag)
            wlr.side[i].fly_adapt = 0 ;
            
             if(wlr.side[i].fly_adapt == 1)//增益落地500ms
              {
               wlr.side[i].fly_flag_cnt = 100; 
             }
             else if(wlr.side[i].fly_adapt == 0 && wlr.side[i].fly_flag_cnt>0)
                wlr.side[i].fly_flag_cnt--;    
        }      
        else {
            wlr.side[i].fly_flag = 0;
            wlr.side[i].fly_cnt = 0;
        }
    }
    
//    state = knn_classify(input_data, 25);
//    wlr.side[0].fly_flag =  state;
//    wlr.side[1].fly_flag =  state;
	//高度选择
    if (wlr.power_flag == 0) {
        if (fabs(wlr.v_ref) > fabs(wlr.v_fdb)) {
            data_limit(&wlr.v_ref,wlr.v_fdb-1.0f,wlr.v_fdb+1.0f);
        }
    }
	if (wlr.high_flag == 2) { //站高高
        wlr.high_set = ramp_calc(&height_ramp, LegLengthHigh2);
        pid_leg_length[0].kp = pid_leg_length[1].kp = pid_p;     
        pid_leg_length[0].ki = pid_leg_length[1].ki = pid_i;
        pid_leg_length[0].kd = pid_leg_length[1].kd = pid_d;
    } else if (wlr.high_flag == 1) { //高
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
            wlr.high_set = 0.30f;
//            wlr.high_set = ramp_calc(&height_ramp, LegLengthHighFly);
        } else {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthHigh);
        } 
        pid_leg_length[0].kp = pid_leg_length[1].kp = pid_p;
        pid_leg_length[0].ki = pid_leg_length[1].ki = pid_i;
        pid_leg_length[0].kd = pid_leg_length[1].kd = pid_d;        
    } else { //正常腿长
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthFly);
        } else {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthNormal);
        }
        pid_leg_length[0].kp = pid_leg_length[1].kp = pid_p;
        pid_leg_length[0].ki = pid_leg_length[1].ki = pid_i;
        pid_leg_length[0].kd = pid_leg_length[1].kd = pid_d;
    }
    //上台阶
    if (wlr.jump_flag == 1) //跳跃起跳状态 先压腿
        {  
        wlr.high_set = 0.15f;
        wlr.jump_cnt++;
        if (wlr.jump_cnt > 250) {
            wlr.jump_cnt = 0;
            wlr.jump_flag = 2;
        }
//             if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
//            wlr.jump_flag = 2;
    } else if (wlr.jump_flag == 2) { //起跳 弹腿
        wlr.high_set = 0.35f;
        pid_leg_length[0].kp = pid_leg_length[1].kp = 1000.0f;
        pid_leg_length[0].ki = pid_leg_length[1].ki = pid_i;
        pid_leg_length[0].kd = pid_leg_length[1].kd = 8000.0f;
        if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
            wlr.jump_flag = 3;
    } else if (wlr.jump_flag == 3) { //收腿
        wlr.high_set = 0.15f;
                if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
            wlr.jump_flag = 4;
    }
        else if(wlr.jump_flag == 4) { //落地
        wlr.high_set = 0.20f;
//        pid_leg_length[0].kd = pid_leg_length[1].kd = 80000.0f;
              wlr.jump_cnt++;
        if (wlr.jump_cnt > 500) {
            wlr.jump_cnt = 0;
            wlr.jump_flag = 5;
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
        } else if (wlr.jump_flag != 0) {
            k_array_fit(K_Array_Leg, vmc[0].L_fdb, vmc[1].L_fdb);
            aMartix_Cover(lqr.K, (float*)K_Array_Leg, 4, 10);	
        } else if (wlr.high_flag == 2) {
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_030, 4, 10);			
        } else {
//            k_array_fit(K_Array_Leg, vmc[0].L_fdb, vmc[1].L_fdb);
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_020, 4, 10);
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
    if(fabs(wlr.v_ref) < 1e-3 && (wlr.jump_flag == 0 || wlr.jump_flag == 3 || wlr.jump_flag == 4))
    {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_adapt;
    } else {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_fdb;
        wlr.s_adapt = wlr.s_fdb;
    }
    if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_adapt = wlr.s_fdb;
    }
//	if (wlr.side[0].fly_flag || wlr.side[1].fly_flag) //腾空
//		wlr.v_ref = 0;
    lqr.X_ref[1] = wlr.v_ref;
    lqr.X_ref[2] = -wlr.yaw_ref;
    lqr.X_ref[3] = -wlr.wz_ref;
    
    aMartix_Add(1, lqr.X_ref, -1, lqr.X_fdb, lqr.X_diff, 10, 1);
    data_limit(&lqr.X_diff[1], -2.0f, 2.0f); //<! 速度窗口
    aMartix_Mul(lqr.K, lqr.X_diff, lqr.U_ref, 4, 10, 1);
    
    //预测下一个时刻的状态
    p_array_fit(P_Array, vmc[0].L_fdb, vmc[1].L_fdb);
    state_predict();
	//虚拟力映射
	for (int i = 0; i < 2; i++) {
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag && wlr.jump_flag==0) {
		wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		}
        else if (wlr.jump_flag == 1 ){
		wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);
        }			
        else if (wlr.jump_flag == 2){
		wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+30.0f;
		} else if (wlr.jump_flag == 3  ){		
              wlr.side[i].Fy =  pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb);            
		}
        else if (wlr.jump_flag == 4){
			wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb);		
        
		}  else	{													                       //常态 跳跃压腿阶段 跳跃落地阶段
          
            if(wlr.side[0].fly_flag_cnt && wlr.side[1].fly_flag_cnt)
						{
            wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb) ;
						}

            else
               wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb) ; 
            
        }
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
