#ifndef __MATH_MATRIX_H
#define __MATH_MATRIX_H

#include "stdint.h"

#define UINT unsigned int     //矩阵索引类型，类型数值范围需大于涉及矩阵元素最大数量

typedef struct
{
	UINT r;//行
	UINT c;//列
	float* array;
} Martix_t;

Martix_t* Martix_Create(UINT r, UINT c);
Martix_t* Martix_Cover(Martix_t* m, float* data);
float* Martix_To_Array(Martix_t* m, float* data);
Martix_t* Martix_Transpose(Martix_t* m, Martix_t* ms);
float Martix_Get(Martix_t* m, UINT i, UINT j);
void Martix_Set(Martix_t* m, UINT i, UINT j, float element);
Martix_t* Martix_nMult(float k, Martix_t* m, Martix_t* ms);
Martix_t* Martix_Add(float sign1, Martix_t* ml, float sign2, Martix_t* mr, Martix_t* ms);
Martix_t* Martix_Mult(Martix_t* ml, Martix_t* mr, Martix_t* ms);
void Martix_print(Martix_t* m);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//为便于keil的硬件仿真直接查看数据，并赋予每一个元素物理意义，设计如下形式的矩阵运算

//mat(m*n)
void aMartix_Cover(float* mat, float* data, UINT m, UINT n);
//ml(m*n) mr(m*n)
void aMartix_Add(float  sign1, float* ml, float sign2, float* mr, float* ms, UINT m, UINT n);
//mr(m*n) ms(m*n)
void aMartix_nMul(float kp, float* mr, float* ms, UINT m, UINT n);
//ml(m*n) mr(n*p) ms(m*p)
void aMartix_Mul(float* ml, float* mr, float* ms, UINT m, UINT n, UINT p);
//mat(m*n) ms(n*m)
void aMartix_Trans(float* mat, float* ms, UINT m, UINT n);
//mat(n*n)
float aMartix_Det(float* mat, UINT n);
// 一维数组形式的矩阵的高斯法求逆
int aMartix_Inv(float *a, UINT n);
//a(m*n) b(n*n) c(m*m)
void aMartix_AXApie(float* a, float* b, float* c, UINT m, UINT n);
//mat(m*n)
void aMartix_print(float* mat, UINT m, UINT n);

#endif
