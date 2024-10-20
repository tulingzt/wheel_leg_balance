#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "quadprog.h"

#define MAX_ITER 1000
#define EPSILON 1e-6

// 简单的二次规划求解器
void quadprog(double *H, double *f, double *A, double *b, double *x, int n, int m) {
    
    double alpha = 0.01; // 学习率
    for (int iter = 0; iter < MAX_ITER; iter++) {
        // 计算梯度
        double grad[n];
        for (int i = 0; i < n; i++) {
            grad[i] = f[i];
            for (int j = 0; j < n; j++) {
                grad[i] += H[i * n + j] * x[j];
            }
        }

        // 更新变量
        for (int i = 0; i < n; i++) {
            x[i] -= alpha * grad[i];
            // 应用约束（简单的投影方法）
            if (x[i] < 0) x[i] = 0; // 示例约束
        }

        // 检查收敛
        double norm = 0;
        for (int i = 0; i < n; i++) {
            norm += grad[i] * grad[i];
        }
        if (sqrt(norm) < EPSILON) break;
    }
}

int main() {
    double H[4] = {2.0, 0.0, 0.0, 2.0};  // 二次项系数矩阵 (2x2)
    double f[2] = {-2.0, -5.0};          // 线性项系数向量
    double x[2] = {0.0, 0.0};            // 初始猜测

    quadprog(H, f, NULL, NULL, x, 2, 0); // 无约束示例

    printf("Optimal solution: x = [%f, %f]\n", x[0], x[1]);
    return 0;
}


//#include "arm_math.h"
//#include <vector>
//#include <iostream>
//#include <osqp.h>  // 确保引入 OSQP 头文件

//bool SolveLinearMPC(const arm_matrix_instance_f32 *matrix_a, const arm_matrix_instance_f32 *matrix_b,
//                    const arm_matrix_instance_f32 *matrix_c, const arm_matrix_instance_f32 *matrix_q,
//                    const arm_matrix_instance_f32 *matrix_r, const arm_matrix_instance_f32 *matrix_lower,
//                    const arm_matrix_instance_f32 *matrix_upper, const arm_matrix_instance_f32 *matrix_initial_state,
//                    const std::vector<arm_matrix_instance_f32> &reference, const float eps,
//                    const int max_iter, std::vector<arm_matrix_instance_f32> *control) {

//    if (matrix_a->rows != matrix_a->cols ||
//        matrix_b->rows != matrix_a->rows ||
//        matrix_lower->rows != matrix_upper->rows) {
//        std::cerr << "一个或多个矩阵的维度不兼容，正在中止。" << std::endl;
//        return false;
//    }

//    unsigned int horizon = reference.size();
//    arm_matrix_instance_f32 matrix_t, matrix_v;
//    arm_matrix_instance_f32 matrix_k, matrix_m, matrix_qq, matrix_rr, matrix_ll, matrix_uu;

//    arm_mat_init_f32(&matrix_t, matrix_b->rows * horizon, 1, nullptr);
//    arm_mat_init_f32(&matrix_v, matrix_b->rows * horizon, 1, nullptr);
//    arm_mat_init_f32(&matrix_k, matrix_b->rows * horizon, matrix_b->cols * horizon, nullptr);
//    arm_mat_init_f32(&matrix_m, matrix_b->rows * horizon, 1, nullptr);
//    arm_mat_init_f32(&matrix_qq, matrix_k.rows, matrix_k.rows, nullptr);
//    arm_mat_init_f32(&matrix_rr, matrix_k.cols, matrix_k.cols, nullptr);
//    arm_mat_init_f32(&matrix_ll, horizon * matrix_lower->rows, 1, nullptr);
//    arm_mat_init_f32(&matrix_uu, horizon * matrix_upper->rows, 1, nullptr);

//    // 更新矩阵 t
//    for (unsigned int j = 0; j < horizon; ++j) {
//        arm_mat_copy(&reference[j], &matrix_t); // 这里需要处理 block 操作
//    }

//    // 计算 A 的幂
//    std::vector<arm_matrix_instance_f32> matrix_a_power(horizon);
//    for (size_t i = 0; i < horizon; ++i) {
//        arm_mat_init_f32(&matrix_a_power[i], matrix_a->rows, matrix_a->cols, nullptr);
//        if (i == 0) {
//            arm_mat_copy(matrix_a, &matrix_a_power[i]);
//        } else {
//            arm_mat_mult_f32(&matrix_a_power[i - 1], matrix_a, &matrix_a_power[i]);
//        }
//    }

//    // 计算 K 矩阵
//    arm_mat_init_f32(&matrix_k, matrix_b->rows * horizon, matrix_b->cols * horizon, nullptr);
//    for (size_t r = 0; r < horizon; ++r) {
//        for (size_t c = 0; c <= r; ++c) {
//            arm_matrix_instance_f32 temp;
//            arm_mat_init_f32(&temp, matrix_b->rows, matrix_b->cols, nullptr);
//            arm_mat_mult_f32(&matrix_a_power[r - c], matrix_b, &temp);
//            // 处理 block 操作，将 temp 放到 matrix_k 的对应位置
//        }
//        arm_mat_copy(matrix_b, &matrix_k); // 处理对角元素
//    }

//    // 计算矩阵 M
//    arm_mat_mult_f32(matrix_a, matrix_initial_state, &matrix_m);
//    arm_mat_add_f32(&matrix_m, matrix_c, &matrix_m);

//    // 填充 Q 和 R 矩阵，以及约束条件
//    for (unsigned int i = 0; i < horizon; ++i) {
//        arm_mat_copy(matrix_lower, &matrix_ll); // 填充下限
//        arm_mat_copy(matrix_upper, &matrix_uu); // 填充上限
//        arm_mat_copy(matrix_q, &matrix_qq); // 填充 Q 矩阵
//        arm_mat_copy(matrix_r, &matrix_rr); // 填充 R 矩阵
//    }

//    // 设置 QP 求解器
//    int nV = matrix_k.cols; // 变量的个数
//    int nC = matrix_ll.rows; // 约束条件的个数

//    // 初始化 OSQP
//    OSQPWorkspace *workspace;
//    OSQPSettings settings;
//    osqp_set_default_settings(&settings);
//    settings.verbose = 0;  // 关闭详细输出

//    // 设置 QP 矩阵
//    c_float *H = new c_float[nV * nV];
//    c_float *g = new c_float[nV];
//    c_float *lb = new c_float[nC];
//    c_float *ub = new c_float[nC];

//    // 填充 H 矩阵
//    for (int i = 0; i < nV; ++i) {
//        for (int j = 0; j < nV; ++j) {
//            H[i * nV + j] = matrix_m1.pData[i * nV + j]; // 将 M1 的数据填入 H
//        }
//    }

//    // 填充 g 向量
//    for (int i = 0; i < nV; ++i) {
//        g[i] = matrix_m2.pData[i]; // 将 M2 的数据填入 g
//    }

//    // 填充约束条件
//    for (int i = 0; i < nC; ++i) {
//        lb[i] = matrix_ll.pData[i]; // 下限
//        ub[i] = matrix_uu.pData[i]; // 上限
//    }

//    // 设置 QP 问题
//    osqp_setup(&workspace, H, g, nV, nC, lb, ub, nullptr, nullptr, &settings);

//    // 求解 QP 问题
//    osqp_solve(workspace);

//    // 获取解
//    c_float *xOpt = new c_float[nV];
//    osqp_get_primal_solution(workspace, xOpt);

//    // 将解填入控制向量
//    for (unsigned int i = 0; i < horizon; ++i) {
//        arm_matrix_instance_f32 control_temp;
//        arm_mat_init_f32(&control_temp, matrix_v.rows / horizon, 1, nullptr);
//        for (int j = 0; j < control_temp.rows; ++j) {
//            control_temp.pData[j] = xOpt[i * control_temp.rows + j]; // 将最优解放入控制向量
//        }
//        (*control)[i] = control_temp; // 保存控制向量
//    }

//    // 释放内存
//    delete[] H;
//    delete[] g;
//    delete[] lb;
//    delete[] ub;
//    delete[] xOpt;
//    osqp_cleanup(workspace);

//    return true; // 返回成功
//}

//#include <stdio.h>
//#include <stdlib.h>
//#include <osqp.h>
//#include <arm_math.h>  // 确保引入 ARM 的数学库

//bool SolveLinearMPC(const arm_matrix_instance_f32 *matrix_a, const arm_matrix_instance_f32 *matrix_b,
//                    const arm_matrix_instance_f32 *matrix_c, const arm_matrix_instance_f32 *matrix_q,
//                    const arm_matrix_instance_f32 *matrix_r, const arm_matrix_instance_f32 *matrix_lower,
//                    const arm_matrix_instance_f32 *matrix_upper, const arm_matrix_instance_f32 *matrix_initial_state,
//                    const arm_matrix_instance_f32 *reference, const float eps,
//                    const int max_iter, arm_matrix_instance_f32 *control, int horizon) {

//    if (matrix_a->rows != matrix_a->cols ||
//        matrix_b->rows != matrix_a->rows ||
//        matrix_lower->rows != matrix_upper->rows) {
//        fprintf(stderr, "一个或多个矩阵的维度不兼容，正在中止。\n");
//        return false;
//    }

//    arm_matrix_instance_f32 matrix_t, matrix_k, matrix_m, matrix_qq, matrix_rr;
//    arm_mat_init_f32(&matrix_t, matrix_b->rows * horizon, 1);
//    arm_mat_init_f32(&matrix_k, matrix_b->rows * horizon, matrix_b->cols * horizon);
//    arm_mat_init_f32(&matrix_m, matrix_b->rows * horizon, 1);
//    arm_mat_init_f32(&matrix_qq, matrix_k.rows, matrix_k.rows);
//    arm_mat_init_f32(&matrix_rr, matrix_k.cols, matrix_k.cols);

//    // 更新矩阵 t
//    for (int j = 0; j < horizon; ++j) {
//        arm_mat_copy(&reference[j], &matrix_t); // 处理矩阵 t
//    }

//    // 计算 A 的幂
//    arm_matrix_instance_f32 *matrix_a_power = (arm_matrix_instance_f32 *)malloc(horizon * sizeof(arm_matrix_instance_f32));
//    for (int i = 0; i < horizon; ++i) {
//        arm_mat_init_f32(&matrix_a_power[i], matrix_a->rows, matrix_a->cols);
//        if (i == 0) {
//            arm_mat_copy(matrix_a, &matrix_a_power[i]);
//        } else {
//            arm_mat_mult_f32(&matrix_a_power[i - 1], matrix_a, &matrix_a_power[i]);
//        }
//    }

//    // 计算 K 矩阵
//    for (int r = 0; r < horizon; ++r) {
//        for (int c = 0; c <= r; ++c) {
//            arm_matrix_instance_f32 temp;
//            arm_mat_init_f32(&temp, matrix_b->rows, matrix_b->cols);
//            arm_mat_mult_f32(&matrix_a_power[r - c], matrix_b, &temp);
//            // 处理 block 操作，将 temp 放到 matrix_k 的对应位置
//        }
//        // 处理对角元素
//    }

//    // 计算矩阵 M
//    arm_mat_mult_f32(matrix_a, matrix_initial_state, &matrix_m);
//    arm_mat_add_f32(&matrix_m, matrix_c, &matrix_m);

//    // 填充 Q 和 R 矩阵
//    for (int i = 0; i < horizon; ++i) {
//        arm_mat_copy(matrix_lower, &matrix_ll); // 填充下限
//        arm_mat_copy(matrix_upper, &matrix_uu); // 填充上限
//        arm_mat_copy(matrix_q, &matrix_qq); // 填充 Q 矩阵
//        arm_mat_copy(matrix_r, &matrix_rr); // 填充 R 矩阵
//    }

//    // 设置 QP 求解器
//    int nV = matrix_k.cols; // 变量的个数
//    int nC = matrix_lower->rows; // 约束条件的个数

//    // 初始化 OSQP
//    OSQPWorkspace *workspace;
//    OSQPSettings settings;
//    osqp_set_default_settings(&settings);
//    settings.verbose = 0; // 关闭详细输出

//    // 设置 QP 矩阵
//    c_float *H = (c_float *)malloc(nV * nV * sizeof(c_float));
//    c_float *g = (c_float *)malloc(nV * sizeof(c_float));
//    c_float *lb = (c_float *)malloc(nC * sizeof(c_float));
//    c_float *ub = (c_float *)malloc(nC * sizeof(c_float));

//    // 填充 H 矩阵
//    for (int i = 0; i < nV; ++i) {
//        for (int j = 0; j < nV; ++j) {
//            H[i * nV + j] = matrix_m1.pData[i * nV + j]; // 将 M1 的数据填入 H
//        }
//    }

//    // 填充 g 向量
//    for (int i = 0; i < nV; ++i) {
//        g[i] = matrix_m2.pData[i]; // 将 M2 的数据填入 g
//    }

//    // 填充约束条件
//    for (int i = 0; i < nC; ++i) {
//        lb[i] = matrix_ll.pData[i]; // 下限
//        ub[i] = matrix_uu.pData[i]; // 上限
//    }

//    // 设置 QP 问题
//    osqp_setup(&workspace, H, g, nV, nC, lb, ub, NULL, NULL, &settings);

//    // 求解 QP 问题
//    osqp_solve(workspace);

//    // 获取解
//    c_float *xOpt = (c_float *)malloc(nV * sizeof(c_float));
//    osqp_get_primal_solution(workspace, xOpt);

//    // 将解填入控制向量
//    for (int i = 0; i < horizon; ++i) {
//        for (int j = 0; j < control->rows; ++j) {
//            control->pData[j] = xOpt[i * control->rows + j]; // 将最优解放入控制向量
//        }
//    }

//    // 释放内存
//    free(H);
//    free(g);
//    free(lb);
//    free(ub);
//    free(xOpt);
//    osqp_cleanup(workspace);
//    free(matrix_a_power);

//    return true; // 返回成功
//}

