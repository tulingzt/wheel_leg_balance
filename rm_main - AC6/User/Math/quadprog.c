#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "quadprog.h"

#define MAX_ITER 1000
#define EPSILON 1e-6

// �򵥵Ķ��ι滮�����
void quadprog(double *H, double *f, double *A, double *b, double *x, int n, int m) {
    
    double alpha = 0.01; // ѧϰ��
    for (int iter = 0; iter < MAX_ITER; iter++) {
        // �����ݶ�
        double grad[n];
        for (int i = 0; i < n; i++) {
            grad[i] = f[i];
            for (int j = 0; j < n; j++) {
                grad[i] += H[i * n + j] * x[j];
            }
        }

        // ���±���
        for (int i = 0; i < n; i++) {
            x[i] -= alpha * grad[i];
            // Ӧ��Լ�����򵥵�ͶӰ������
            if (x[i] < 0) x[i] = 0; // ʾ��Լ��
        }

        // �������
        double norm = 0;
        for (int i = 0; i < n; i++) {
            norm += grad[i] * grad[i];
        }
        if (sqrt(norm) < EPSILON) break;
    }
}

int main() {
    double H[4] = {2.0, 0.0, 0.0, 2.0};  // ������ϵ������ (2x2)
    double f[2] = {-2.0, -5.0};          // ������ϵ������
    double x[2] = {0.0, 0.0};            // ��ʼ�²�

    quadprog(H, f, NULL, NULL, x, 2, 0); // ��Լ��ʾ��

    printf("Optimal solution: x = [%f, %f]\n", x[0], x[1]);
    return 0;
}


//#include "arm_math.h"
//#include <vector>
//#include <iostream>
//#include <osqp.h>  // ȷ������ OSQP ͷ�ļ�

//bool SolveLinearMPC(const arm_matrix_instance_f32 *matrix_a, const arm_matrix_instance_f32 *matrix_b,
//                    const arm_matrix_instance_f32 *matrix_c, const arm_matrix_instance_f32 *matrix_q,
//                    const arm_matrix_instance_f32 *matrix_r, const arm_matrix_instance_f32 *matrix_lower,
//                    const arm_matrix_instance_f32 *matrix_upper, const arm_matrix_instance_f32 *matrix_initial_state,
//                    const std::vector<arm_matrix_instance_f32> &reference, const float eps,
//                    const int max_iter, std::vector<arm_matrix_instance_f32> *control) {

//    if (matrix_a->rows != matrix_a->cols ||
//        matrix_b->rows != matrix_a->rows ||
//        matrix_lower->rows != matrix_upper->rows) {
//        std::cerr << "һ�����������ά�Ȳ����ݣ�������ֹ��" << std::endl;
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

//    // ���¾��� t
//    for (unsigned int j = 0; j < horizon; ++j) {
//        arm_mat_copy(&reference[j], &matrix_t); // ������Ҫ���� block ����
//    }

//    // ���� A ����
//    std::vector<arm_matrix_instance_f32> matrix_a_power(horizon);
//    for (size_t i = 0; i < horizon; ++i) {
//        arm_mat_init_f32(&matrix_a_power[i], matrix_a->rows, matrix_a->cols, nullptr);
//        if (i == 0) {
//            arm_mat_copy(matrix_a, &matrix_a_power[i]);
//        } else {
//            arm_mat_mult_f32(&matrix_a_power[i - 1], matrix_a, &matrix_a_power[i]);
//        }
//    }

//    // ���� K ����
//    arm_mat_init_f32(&matrix_k, matrix_b->rows * horizon, matrix_b->cols * horizon, nullptr);
//    for (size_t r = 0; r < horizon; ++r) {
//        for (size_t c = 0; c <= r; ++c) {
//            arm_matrix_instance_f32 temp;
//            arm_mat_init_f32(&temp, matrix_b->rows, matrix_b->cols, nullptr);
//            arm_mat_mult_f32(&matrix_a_power[r - c], matrix_b, &temp);
//            // ���� block �������� temp �ŵ� matrix_k �Ķ�Ӧλ��
//        }
//        arm_mat_copy(matrix_b, &matrix_k); // ����Խ�Ԫ��
//    }

//    // ������� M
//    arm_mat_mult_f32(matrix_a, matrix_initial_state, &matrix_m);
//    arm_mat_add_f32(&matrix_m, matrix_c, &matrix_m);

//    // ��� Q �� R �����Լ�Լ������
//    for (unsigned int i = 0; i < horizon; ++i) {
//        arm_mat_copy(matrix_lower, &matrix_ll); // �������
//        arm_mat_copy(matrix_upper, &matrix_uu); // �������
//        arm_mat_copy(matrix_q, &matrix_qq); // ��� Q ����
//        arm_mat_copy(matrix_r, &matrix_rr); // ��� R ����
//    }

//    // ���� QP �����
//    int nV = matrix_k.cols; // �����ĸ���
//    int nC = matrix_ll.rows; // Լ�������ĸ���

//    // ��ʼ�� OSQP
//    OSQPWorkspace *workspace;
//    OSQPSettings settings;
//    osqp_set_default_settings(&settings);
//    settings.verbose = 0;  // �ر���ϸ���

//    // ���� QP ����
//    c_float *H = new c_float[nV * nV];
//    c_float *g = new c_float[nV];
//    c_float *lb = new c_float[nC];
//    c_float *ub = new c_float[nC];

//    // ��� H ����
//    for (int i = 0; i < nV; ++i) {
//        for (int j = 0; j < nV; ++j) {
//            H[i * nV + j] = matrix_m1.pData[i * nV + j]; // �� M1 ���������� H
//        }
//    }

//    // ��� g ����
//    for (int i = 0; i < nV; ++i) {
//        g[i] = matrix_m2.pData[i]; // �� M2 ���������� g
//    }

//    // ���Լ������
//    for (int i = 0; i < nC; ++i) {
//        lb[i] = matrix_ll.pData[i]; // ����
//        ub[i] = matrix_uu.pData[i]; // ����
//    }

//    // ���� QP ����
//    osqp_setup(&workspace, H, g, nV, nC, lb, ub, nullptr, nullptr, &settings);

//    // ��� QP ����
//    osqp_solve(workspace);

//    // ��ȡ��
//    c_float *xOpt = new c_float[nV];
//    osqp_get_primal_solution(workspace, xOpt);

//    // ���������������
//    for (unsigned int i = 0; i < horizon; ++i) {
//        arm_matrix_instance_f32 control_temp;
//        arm_mat_init_f32(&control_temp, matrix_v.rows / horizon, 1, nullptr);
//        for (int j = 0; j < control_temp.rows; ++j) {
//            control_temp.pData[j] = xOpt[i * control_temp.rows + j]; // �����Ž�����������
//        }
//        (*control)[i] = control_temp; // �����������
//    }

//    // �ͷ��ڴ�
//    delete[] H;
//    delete[] g;
//    delete[] lb;
//    delete[] ub;
//    delete[] xOpt;
//    osqp_cleanup(workspace);

//    return true; // ���سɹ�
//}

//#include <stdio.h>
//#include <stdlib.h>
//#include <osqp.h>
//#include <arm_math.h>  // ȷ������ ARM ����ѧ��

//bool SolveLinearMPC(const arm_matrix_instance_f32 *matrix_a, const arm_matrix_instance_f32 *matrix_b,
//                    const arm_matrix_instance_f32 *matrix_c, const arm_matrix_instance_f32 *matrix_q,
//                    const arm_matrix_instance_f32 *matrix_r, const arm_matrix_instance_f32 *matrix_lower,
//                    const arm_matrix_instance_f32 *matrix_upper, const arm_matrix_instance_f32 *matrix_initial_state,
//                    const arm_matrix_instance_f32 *reference, const float eps,
//                    const int max_iter, arm_matrix_instance_f32 *control, int horizon) {

//    if (matrix_a->rows != matrix_a->cols ||
//        matrix_b->rows != matrix_a->rows ||
//        matrix_lower->rows != matrix_upper->rows) {
//        fprintf(stderr, "һ�����������ά�Ȳ����ݣ�������ֹ��\n");
//        return false;
//    }

//    arm_matrix_instance_f32 matrix_t, matrix_k, matrix_m, matrix_qq, matrix_rr;
//    arm_mat_init_f32(&matrix_t, matrix_b->rows * horizon, 1);
//    arm_mat_init_f32(&matrix_k, matrix_b->rows * horizon, matrix_b->cols * horizon);
//    arm_mat_init_f32(&matrix_m, matrix_b->rows * horizon, 1);
//    arm_mat_init_f32(&matrix_qq, matrix_k.rows, matrix_k.rows);
//    arm_mat_init_f32(&matrix_rr, matrix_k.cols, matrix_k.cols);

//    // ���¾��� t
//    for (int j = 0; j < horizon; ++j) {
//        arm_mat_copy(&reference[j], &matrix_t); // ������� t
//    }

//    // ���� A ����
//    arm_matrix_instance_f32 *matrix_a_power = (arm_matrix_instance_f32 *)malloc(horizon * sizeof(arm_matrix_instance_f32));
//    for (int i = 0; i < horizon; ++i) {
//        arm_mat_init_f32(&matrix_a_power[i], matrix_a->rows, matrix_a->cols);
//        if (i == 0) {
//            arm_mat_copy(matrix_a, &matrix_a_power[i]);
//        } else {
//            arm_mat_mult_f32(&matrix_a_power[i - 1], matrix_a, &matrix_a_power[i]);
//        }
//    }

//    // ���� K ����
//    for (int r = 0; r < horizon; ++r) {
//        for (int c = 0; c <= r; ++c) {
//            arm_matrix_instance_f32 temp;
//            arm_mat_init_f32(&temp, matrix_b->rows, matrix_b->cols);
//            arm_mat_mult_f32(&matrix_a_power[r - c], matrix_b, &temp);
//            // ���� block �������� temp �ŵ� matrix_k �Ķ�Ӧλ��
//        }
//        // ����Խ�Ԫ��
//    }

//    // ������� M
//    arm_mat_mult_f32(matrix_a, matrix_initial_state, &matrix_m);
//    arm_mat_add_f32(&matrix_m, matrix_c, &matrix_m);

//    // ��� Q �� R ����
//    for (int i = 0; i < horizon; ++i) {
//        arm_mat_copy(matrix_lower, &matrix_ll); // �������
//        arm_mat_copy(matrix_upper, &matrix_uu); // �������
//        arm_mat_copy(matrix_q, &matrix_qq); // ��� Q ����
//        arm_mat_copy(matrix_r, &matrix_rr); // ��� R ����
//    }

//    // ���� QP �����
//    int nV = matrix_k.cols; // �����ĸ���
//    int nC = matrix_lower->rows; // Լ�������ĸ���

//    // ��ʼ�� OSQP
//    OSQPWorkspace *workspace;
//    OSQPSettings settings;
//    osqp_set_default_settings(&settings);
//    settings.verbose = 0; // �ر���ϸ���

//    // ���� QP ����
//    c_float *H = (c_float *)malloc(nV * nV * sizeof(c_float));
//    c_float *g = (c_float *)malloc(nV * sizeof(c_float));
//    c_float *lb = (c_float *)malloc(nC * sizeof(c_float));
//    c_float *ub = (c_float *)malloc(nC * sizeof(c_float));

//    // ��� H ����
//    for (int i = 0; i < nV; ++i) {
//        for (int j = 0; j < nV; ++j) {
//            H[i * nV + j] = matrix_m1.pData[i * nV + j]; // �� M1 ���������� H
//        }
//    }

//    // ��� g ����
//    for (int i = 0; i < nV; ++i) {
//        g[i] = matrix_m2.pData[i]; // �� M2 ���������� g
//    }

//    // ���Լ������
//    for (int i = 0; i < nC; ++i) {
//        lb[i] = matrix_ll.pData[i]; // ����
//        ub[i] = matrix_uu.pData[i]; // ����
//    }

//    // ���� QP ����
//    osqp_setup(&workspace, H, g, nV, nC, lb, ub, NULL, NULL, &settings);

//    // ��� QP ����
//    osqp_solve(workspace);

//    // ��ȡ��
//    c_float *xOpt = (c_float *)malloc(nV * sizeof(c_float));
//    osqp_get_primal_solution(workspace, xOpt);

//    // ���������������
//    for (int i = 0; i < horizon; ++i) {
//        for (int j = 0; j < control->rows; ++j) {
//            control->pData[j] = xOpt[i * control->rows + j]; // �����Ž�����������
//        }
//    }

//    // �ͷ��ڴ�
//    free(H);
//    free(g);
//    free(lb);
//    free(ub);
//    free(xOpt);
//    osqp_cleanup(workspace);
//    free(matrix_a_power);

//    return true; // ���سɹ�
//}

