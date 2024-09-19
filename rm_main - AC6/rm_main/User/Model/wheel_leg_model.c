#include "wheel_leg_model.h"
#include "math_lib.h"
#include "kalman_filter.h"

kalman_filter_t kal_gnd_roll;
kalman_filter_t kal_v0_fdb, kal_w0_fdb;
twm_t twm;
tlm_t tlm;

/*
 * @brief     两腿模型初始化
 * @param[in] tlm  : 两腿模型实例
 * @param[in] Lmax : 最大腿长
 * @param[in] Lmin : 最小腿长
 * @param[in] width: 两腿间距
 * @retval    void
 */
void tlm_init(tlm_t *tlm, float Lmax, float Lmin, float width)
{
    tlm->leg_max = Lmax;
    tlm->leg_min = Lmin;
    tlm->leg_width = width;
    kalman_filter_init(&kal_gnd_roll, 1, 0, 1);
    kal_gnd_roll.A_data[0] = 1;
    kal_gnd_roll.H_data[0] = 1;
    kal_gnd_roll.Q_data[0] = 1;
    kal_gnd_roll.R_data[0] = 100;
}

/*
 * @brief     两腿模型 计算地面roll角
 * @param[in] tlm     : 两腿模型实例
 * @param[in] imu_roll: 陀螺仪roll角度
 * @param[in] Ll_fdb  : 左腿长度
 * @param[in] Lr_fdb  : 右腿长度
 * @retval    void
 */
void tlm_gnd_roll_calc(tlm_t *tlm, float imu_roll, float Ll_fdb, float Lr_fdb)
{
    tlm->l_fdb[0] = Ll_fdb;
    tlm->l_fdb[1] = Lr_fdb;
    tlm->imu_roll_fdb = imu_roll;
    tlm->leg_roll_fdb = atanf((tlm->l_fdb[0] - tlm->l_fdb[1]) / tlm->leg_width);
    tlm->gnd_roll_fdb = tlm->imu_roll_fdb - tlm->leg_roll_fdb;

    kal_gnd_roll.measured_vector[0] = tlm->gnd_roll_fdb;
    kalman_filter_update(&kal_gnd_roll);
    tlm->gnd_roll_fdb = kal_gnd_roll.filter_vector[0];
}

/*
 * @brief     两腿模型 计算两腿长度
 * @param[in] tlm           : 两腿模型实例
 * @param[in] leg_length_ref: 目标高度
 * @param[in] imu_roll_ref  : 目标roll角度
 * @retval    void
 */
void tlm_leg_length_cacl(tlm_t* tlm, float leg_length_ref, float imu_roll_ref)
{
    tlm->leg_length_ref = leg_length_ref;
    //计算腿长补偿值
    if (ABS(tlm->gnd_roll_fdb) > 0.1745f)	//当倾斜角大于10°时，roll控制为水平
        tlm->imu_roll_ref = 0;
    else                                //当在接近水平的地面上时，进行过弯补偿/翻滚角自定控制
        tlm->imu_roll_ref = imu_roll_ref;
    tlm->leg_length_offset = tlm->leg_width * tanf(tlm->imu_roll_ref - tlm->gnd_roll_fdb);//计算补偿腿长
    //腿长设定值计算
    tlm->l_ref[0] = tlm->leg_length_ref + tlm->leg_length_offset / 2;
    tlm->l_ref[1] = tlm->leg_length_ref - tlm->leg_length_offset / 2;
    //腿长设定值超限分配处理 以保持角度为主，超限改变目标高度但不能超过限幅值
    if (ABS(tlm->leg_length_offset) >= tlm->leg_max - tlm->leg_min) {//角度过大，超过可调整范围，用限幅值
        if (tlm->leg_length_offset > 0) {
            tlm->l_ref[0] = tlm->leg_max;
            tlm->l_ref[1] = tlm->leg_min;
        } else {
            tlm->l_ref[0] = tlm->leg_min;
            tlm->l_ref[1] = tlm->leg_max;
        }
    } else {//在可调整范围内，以保持角度为主
        if (tlm->l_ref[0] > tlm->leg_max) {          //左腿长超上限
            tlm->leg_length_err = tlm->l_ref[0] - tlm->leg_max;
            tlm->l_ref[1] -= tlm->leg_length_err;
            tlm->l_ref[0] = tlm->leg_max;
        } else if (tlm->l_ref[1] > tlm->leg_max) {   //右腿长超上限
            tlm->leg_length_err = tlm->l_ref[1] - tlm->leg_max;
            tlm->l_ref[0] -= tlm->leg_length_err;
            tlm->l_ref[1] = tlm->leg_max;
        } else if (tlm->l_ref[0] < tlm->leg_min) {   //左腿长超下限
            tlm->leg_length_err = tlm->leg_min - tlm->l_ref[0];
            tlm->l_ref[1] += tlm->leg_length_err;
            tlm->l_ref[0] = tlm->leg_min;
        } else if (tlm->l_ref[1] < tlm->leg_min) {   //右腿长超下限
            tlm->leg_length_err = tlm->leg_min - tlm->l_ref[1];
            tlm->l_ref[0] += tlm->leg_length_err;
            tlm->l_ref[1] = tlm->leg_min;
        }
    }
}

/*
 * @brief     两轮模型初始化
 * @param[in] twm   : 两轮模型实例
 * @param[in] width : 两轮间距
 * @param[in] radius: 轮子半径
 * @retval    void
 */
void twm_init(twm_t* twm, float width, float radius)
{
    twm->wheel_width = width;
    twm->wheel_radius = radius;
    kalman_filter_init(&kal_v0_fdb, 1, 0, 1);
    kal_v0_fdb.A_data[0] = 1;
    kal_v0_fdb.H_data[0] = 1;
    kal_v0_fdb.Q_data[0] = 1;
    kal_v0_fdb.R_data[0] = 100;
    kalman_filter_init(&kal_w0_fdb, 1, 0, 1);
    kal_w0_fdb.A_data[0] = 1;
    kal_w0_fdb.H_data[0] = 1;
    kal_w0_fdb.Q_data[0] = 1;
    kal_w0_fdb.R_data[0] = 100;
}

/*
 * @brief     两轮模型反馈数据计算
 * @param[in] twm   : 两轮模型实例
 * @param[in] wl_fdb: 左轮角速度
 * @param[in] wr_fdb: 右轮角速度
 * @param[in] imu_wz: 陀螺仪z轴角速度
 * @retval    void
 */
void twm_feedback_calc(twm_t* twm, float wl_fdb, float wr_fdb, float imu_wz)
{
    twm->w_fdb[0] = wl_fdb;
    twm->w_fdb[1] = wr_fdb;
    kal_w0_fdb.measured_vector[0] = imu_wz;
    kalman_filter_update(&kal_w0_fdb);
    twm->w0_imu_fdb = kal_w0_fdb.filter_vector[0];
    
    twm->v_fdb[0] = -twm->w_fdb[0] * twm->wheel_radius;
    twm->v_fdb[1] = -twm->w_fdb[1] * twm->wheel_radius;
    twm->w0_fdb = (twm->v_fdb[1] - twm->v_fdb[0]) / twm->wheel_width;
    kal_v0_fdb.measured_vector[0] = (twm->v_fdb[0] + twm->v_fdb[1]) / 2;
    kalman_filter_update(&kal_v0_fdb);
    twm->v0_fdb = kal_v0_fdb.filter_vector[0];
    
    if (ABS(twm->w0_imu_fdb) < 1e-2f || ABS(twm->v0_fdb) < 0.3f) {//转弯半径过大，向心加速度为零
        twm->r_fdb = 0;
        twm->a_fdb = 0;
    } else {
        twm->r_fdb = -twm->v0_fdb / twm->w0_imu_fdb;
        twm->a_fdb  = powf(twm->w0_imu_fdb, 2) * twm->r_fdb;
    }
    twm->gravity_compensate_angle = -atanf(twm->a_fdb/GRAVITY); // 补偿过大容易失控
}

/*
 * @brief     两轮模型期望数据计算
 * @param[in] twm   : 两轮模型实例
 * @param[in] v0_ref: 期望前进速度
 * @param[in] w0_ref: 期望z轴角速度
 * @retval    void
 */
void twm_reference_calc(twm_t* twm, float v0_ref, float w0_ref)
{
    twm->v0_ref = v0_ref;
    twm->w0_ref = w0_ref;

    twm->v_ref[0] = twm->v0_ref - twm->wheel_width * twm->w0_ref / 2;
    twm->v_ref[1] = twm->v0_ref + twm->wheel_width * twm->w0_ref / 2;

    twm->w_ref[0] = -twm->v_ref[0] / twm->wheel_radius;
    twm->w_ref[1] = -twm->v_ref[1] / twm->wheel_radius;

    if (ABS(twm->w0_ref) < 1e-2f) {//转弯半径过大，向心加速度为零
        twm->r_ref = 0;
        twm->a_ref = 0;
    } else {
        twm->r_ref = -twm->v0_ref / twm->w0_ref;
        twm->a_ref = powf(twm->w0_ref, 2) * twm->r_ref;
    }
}
