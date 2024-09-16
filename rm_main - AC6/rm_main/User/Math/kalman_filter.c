/**
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */
#include "kalman_filter.h"

uint16_t sizeof_var = sizeof(float);

/*
 * @brief     卡尔曼滤波器初始化
 * @param[in] kf:     卡尔曼滤波器结构体
 * @param[in] x_size: 状态向量长度
 * @param[in] u_size: 控制向量长度
 * @param[in] z_size: 测量向量长度
 * @retval    void
 */
void kalman_filter_init(kalman_filter_t *kf, uint8_t x_size, uint8_t u_size, uint8_t z_size)
{
    kf->x_size = x_size;
    kf->u_size = u_size;
    kf->z_size = z_size;
    //外部数据接口初始化
    kf->filter_vector = (float *)user_malloc(sizeof_var * x_size);
    memset(kf->filter_vector, 0, sizeof_var * x_size);
    if (u_size != 0) {
        kf->control_vector = (float *)user_malloc(sizeof_var * u_size);
        memset(kf->control_vector, 0, sizeof_var * u_size);
    }
    kf->measured_vector = (float *)user_malloc(sizeof_var * z_size);
    memset(kf->measured_vector, 0, sizeof_var * z_size);
    kf->min_variance = (float *)user_malloc(sizeof_var * x_size);
    memset(kf->min_variance, 0, sizeof_var * x_size);
    //状态向量初始化 x(k|k) x(k|k-1)
    kf->xhat_data = (float *)user_malloc(sizeof_var * x_size);
    memset(kf->xhat_data, 0, sizeof_var * x_size);
    mat_init(&kf->xhat, kf->x_size, 1, (float *)kf->xhat_data);
    kf->xhatminus_data = (float *)user_malloc(sizeof_var * x_size);
    memset(kf->xhatminus_data, 0, sizeof_var * x_size);
    mat_init(&kf->xhatminus, kf->x_size, 1, (float *)kf->xhatminus_data);
    //控制向量初始化 u
    if (u_size != 0) {
        kf->u_data = (float *)user_malloc(sizeof_var * u_size);
        memset(kf->u_data, 0, sizeof_var * u_size);
        mat_init(&kf->u, kf->u_size, 1, (float *)kf->u_data);
    }
    //测量向量初始化 z
    kf->z_data = (float *)user_malloc(sizeof_var * z_size);
    memset(kf->z_data, 0, sizeof_var * z_size);
    mat_init(&kf->z, kf->z_size, 1, (float *)kf->z_data);
    //协方差矩阵初始化 P(k|k) P(k|k-1)
    kf->P_data = (float *)user_malloc(sizeof_var * x_size * x_size);
    memset(kf->P_data, 0, sizeof_var * x_size * x_size);
    mat_init(&kf->P, kf->x_size, kf->x_size, (float *)kf->P_data);
    kf->Pminus_data = (float *)user_malloc(sizeof_var * x_size * x_size);
    memset(kf->Pminus_data, 0, sizeof_var * x_size * x_size);
    mat_init(&kf->Pminus, kf->x_size, kf->x_size, (float *)kf->Pminus_data);
    //状态转移矩阵初始化 A AT
    kf->A_data = (float *)user_malloc(sizeof_var * x_size * x_size);
    memset(kf->A_data, 0, sizeof_var * x_size * x_size);
    mat_init(&kf->A, kf->x_size, kf->x_size, (float *)kf->A_data);
    kf->AT_data = (float *)user_malloc(sizeof_var * x_size * x_size);
    memset(kf->AT_data, 0, sizeof_var * x_size * x_size);
    mat_init(&kf->AT, kf->x_size, kf->x_size, (float *)kf->AT_data);
    //控制转移矩阵初始化 B
    if (u_size != 0) {
        kf->B_data = (float *)user_malloc(sizeof_var * x_size * u_size);
        memset(kf->B_data, 0, sizeof_var * x_size * u_size);
        mat_init(&kf->B, kf->x_size, kf->u_size, (float *)kf->B_data);
    }
    //测量转移向量初始化 H HT
    kf->H_data = (float *)user_malloc(sizeof_var * z_size * x_size);
    memset(kf->H_data, 0, sizeof_var * z_size * x_size);
    mat_init(&kf->H, kf->z_size, kf->x_size, (float *)kf->H_data);
    kf->HT_data = (float *)user_malloc(sizeof_var * x_size * z_size);
    memset(kf->HT_data, 0, sizeof_var * x_size * z_size);
    mat_init(&kf->HT, kf->x_size, kf->z_size, (float *)kf->HT_data);
    //过程噪声协方差矩阵初始化 Q
    kf->Q_data = (float *)user_malloc(sizeof_var * x_size * x_size);
    memset(kf->Q_data, 0, sizeof_var * x_size * x_size);
    mat_init(&kf->Q, kf->x_size, kf->x_size, (float *)kf->Q_data);
    //测量噪声协方差矩阵初始化 R
    kf->R_data = (float *)user_malloc(sizeof_var * z_size * z_size);
    memset(kf->R_data, 0, sizeof_var * z_size * z_size);
    mat_init(&kf->R, kf->z_size, kf->z_size, (float *)kf->R_data);
    //卡尔曼增益初始化 K
    kf->K_data = (float *)user_malloc(sizeof_var * x_size * z_size);
    memset(kf->K_data, 0, sizeof_var * x_size * z_size);
    mat_init(&kf->K, kf->x_size, kf->z_size, (float *)kf->K_data);
    //中间变量初始化
    kf->S_data = (float *)user_malloc(sizeof_var * kf->x_size * kf->x_size);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_var * kf->x_size * kf->x_size);
    kf->temp_matrix_data2 = (float *)user_malloc(sizeof_var * kf->x_size * kf->x_size);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_var * kf->x_size);
    kf->temp_vector_data2 = (float *)user_malloc(sizeof_var * kf->x_size);
    mat_init(&kf->S, kf->x_size, kf->x_size, (float *)kf->S_data);
    mat_init(&kf->temp_matrix1, kf->x_size, kf->x_size, (float *)kf->temp_matrix_data1);
    mat_init(&kf->temp_matrix2, kf->x_size, kf->x_size, (float *)kf->temp_matrix_data2);
    mat_init(&kf->temp_vector1, kf->x_size, 1, (float *)kf->temp_vector_data1);
    mat_init(&kf->temp_vector2, kf->x_size, 1, (float *)kf->temp_vector_data2);
    
    kf->skip_eq1 = 0;
    kf->skip_eq2 = 0;
    kf->skip_eq3 = 0;
    kf->skip_eq4 = 0;
    kf->skip_eq5 = 0;
}

/*
 * @brief     获取控制和测量信息
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 */
void kalman_filter_measure_update(kalman_filter_t *kf)
{
    memcpy(kf->z_data, kf->measured_vector, sizeof_var * kf->z_size);
    memset(kf->measured_vector, 0, sizeof_var * kf->z_size);
    if (kf->u_size != 0) {
        memcpy(kf->u_data, kf->control_vector, sizeof_var * kf->u_size);
        memset(kf->control_vector, 0, sizeof_var * kf->u_size);
    }
}

/*
 * @brief     先验估计
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 * @note      xhat'(k)=A·xhat(k-1)+B·u
 */
void kalman_filter_xhatminus_update(kalman_filter_t *kf)
{
    if (!kf->skip_eq1) {
        if (kf->u_size > 0) {
            kf->temp_vector1.numRows = kf->x_size;
            kf->temp_vector1.numCols = 1;
            kf->mat_status = mat_mult(&kf->A, &kf->xhat, &kf->temp_vector1);
            kf->temp_vector2.numRows = kf->x_size;
            kf->temp_vector2.numCols = 1;
            kf->mat_status = mat_mult(&kf->B, &kf->u, &kf->temp_vector2);
            kf->mat_status = mat_add(&kf->temp_vector1, &kf->temp_vector2, &kf->xhatminus);
        } else {
            kf->mat_status = mat_mult(&kf->A, &kf->xhat, &kf->xhatminus);
        }
    }
    if (kf->user_func1_f != NULL)
        kf->user_func1_f(kf);
}

/*
 * @brief     预测更新
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 * @note      P'(k)=A·P(k-1)·AT + Q
 */
void kalman_filter_Pminus_update(kalman_filter_t *kf)
{
    if (!kf->skip_eq2) {
        kf->mat_status = mat_trans(&kf->A, &kf->AT);
        kf->mat_status = mat_mult(&kf->A, &kf->P, &kf->Pminus);
        kf->temp_matrix1.numRows = kf->Pminus.numRows;
        kf->temp_matrix1.numCols = kf->AT.numCols;
        kf->mat_status = mat_mult(&kf->Pminus, &kf->AT, &kf->temp_matrix1);
        kf->mat_status = mat_add(&kf->temp_matrix1, &kf->Q, &kf->Pminus);
    }
    if (kf->user_func2_f != NULL)
        kf->user_func2_f(kf);
}

/*
 * @brief     量测更新
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 * @note      K(k)=P'(k)·HT/(H·P'(k)·HT+R)
 */
void kalman_filter_K_update(kalman_filter_t *kf)
{
    if (!kf->skip_eq3) {
        kf->mat_status = mat_trans(&kf->H, &kf->HT);
        kf->temp_matrix1.numRows = kf->H.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->mat_status = mat_mult(&kf->H, &kf->Pminus, &kf->temp_matrix1);
        kf->temp_matrix2.numRows = kf->temp_matrix1.numRows;
        kf->temp_matrix2.numCols = kf->HT.numCols;
        kf->mat_status = mat_mult(&kf->temp_matrix1, &kf->HT, &kf->temp_matrix2);
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->mat_status = mat_add(&kf->temp_matrix2, &kf->R, &kf->S);
        kf->mat_status = mat_inv(&kf->S, &kf->temp_matrix2);
        kf->temp_matrix1.numRows = kf->Pminus.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->mat_status = mat_mult(&kf->Pminus, &kf->HT, &kf->temp_matrix1);
        kf->mat_status = mat_mult(&kf->temp_matrix1, &kf->temp_matrix2, &kf->K);
    }
    if (kf->user_func3_f != NULL)
        kf->user_func3_f(kf);
}

/*
 * @brief     融合
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 * @note      xhat(k)=xhat'(k)+K(k)·(z(k)-H·xhat'(k))
 */
void kalman_filter_xhat_update(kalman_filter_t *kf)
{
    if (!kf->skip_eq4) {
        kf->temp_vector1.numRows = kf->H.numRows;
        kf->temp_vector1.numCols = 1;
        kf->mat_status = mat_mult(&kf->H, &kf->xhatminus, &kf->temp_vector1);
        kf->temp_vector2.numRows = kf->z.numRows;
        kf->temp_vector2.numCols = 1;
        kf->mat_status = mat_sub(&kf->z, &kf->temp_vector1, &kf->temp_vector2);
        kf->temp_vector1.numRows = kf->K.numRows;
        kf->temp_vector1.numCols = 1;
        kf->mat_status = mat_mult(&kf->K, &kf->temp_vector2, &kf->temp_vector1);
        kf->mat_status = mat_add(&kf->xhatminus, &kf->temp_vector1, &kf->xhat);
    }
    if (kf->user_func4_f != NULL)
        kf->user_func4_f(kf);
}

/*
 * @brief     修正方差
 * @param[in] kf: 卡尔曼滤波器结构体
 * @retval    void
 * @note      P(k)=(I-K(k)·H)·P'(k) ==> P(k)=P'(k)-K(k)·H·P'(k)
 */
void kalman_filter_P_update(kalman_filter_t *kf)
{
    if (!kf->skip_eq5) {
        kf->temp_matrix1.numRows = kf->K.numRows;
        kf->temp_matrix1.numCols = kf->H.numCols;
        kf->mat_status = mat_mult(&kf->K, &kf->H, &kf->temp_matrix1);
        kf->temp_matrix2.numRows = kf->temp_matrix1.numRows;
        kf->temp_matrix2.numCols = kf->Pminus.numCols;
        kf->mat_status = mat_mult(&kf->temp_matrix1, &kf->Pminus, &kf->temp_matrix2);
        kf->mat_status = mat_sub(&kf->Pminus, &kf->temp_matrix2, &kf->P);
    }
    if (kf->user_func5_f != NULL)
        kf->user_func5_f(kf);
}

/*
 * @brief     卡尔曼滤波
 * @param[in] kf: 卡尔曼滤波器结构体
 * @note      P(k)=(1-K(k)·H)·P'(k) ==> P(k)=P'(k)-K(k)·H·P'(k)
 */
void *kalman_filter_update(kalman_filter_t *kf)
{
    kalman_filter_measure_update(kf);
    kalman_filter_xhatminus_update(kf);
    kalman_filter_Pminus_update(kf);
    kalman_filter_K_update(kf);
    kalman_filter_xhat_update(kf);
    kalman_filter_P_update(kf);
    for (uint8_t i = 0; i < kf->x_size; i++) {
        if (kf->P_data[i * kf->x_size + i] < kf->min_variance[i])
            kf->P_data[i * kf->x_size + i] = kf->min_variance[i];
    }
    if (kf->user_func6_f != NULL)
        kf->user_func6_f(kf);
    memcpy(kf->filter_vector, kf->xhat_data, sizeof_var * kf->x_size);
    return kf->filter_vector;
}
