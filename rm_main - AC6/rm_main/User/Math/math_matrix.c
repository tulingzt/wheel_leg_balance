#include "math_matrix.h"
#include "stm32h7xx.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

//空矩阵创建
Martix_t* Martix_Create(UINT r, UINT c)
{
	Martix_t* matrix = (Martix_t*)malloc(sizeof(Martix_t));
	assert_param(matrix);
	memset(matrix, 0, sizeof(Martix_t));
	matrix->r = r;
	matrix->c = c;
	matrix->array = (float*)calloc(r * c, sizeof(float));
	assert_param(matrix->array);
	return matrix;
}

//给矩阵整体赋值
Martix_t* Martix_Cover(Martix_t* m, float* data)
{
	assert_param(data);
	for(UINT i = 0, k = 0; i < m->r; ++i)
		for(UINT j = 0; j < m->c; ++j)
			m->array[i * m->c + j] = data[k++];
	return m;
}

//将矩阵转化为一维数组
float* Martix_To_Array(Martix_t* m, float* data)
{
	assert_param(data);
	for(UINT i = 0, k = 0; i < m->r; ++i)
		for(UINT j = 0; j < m->c; ++j)
			data[k++] = m->array[i * m->c + j];
	return data;
}

//矩阵转置
Martix_t* Martix_Transpose(Martix_t* m, Martix_t* ms)
{
	UINT i, j;
	for(i = 0; i < m->r; ++i)
		for(j = 0; j < m->c; ++j)
			ms->array[j * m->r + i] = m->array[i * m->c + j];
	return ms;
}

//获取矩阵元素
float Martix_Get(Martix_t* m, UINT i, UINT j)
{
	assert_param(i <= m->r && j <= m->c);
	i -= 1;
	j -= 1;
	return m->array[i * m->c + j];
}

//赋值矩阵元素
void Martix_Set(Martix_t* m, UINT i, UINT j, float element)
{
	assert_param(i <= m->r && j <= m->c);
	i -= 1;
	j -= 1;
	m->array[i * m->c + j] = element;
}

//矩阵数乘
Martix_t* Martix_nMult(float k, Martix_t* m, Martix_t* ms)
{
	assert_param(m && ms);
	Martix_t* res;
	for(UINT i = 0; i < m->r * m->c; ++i)
	{
		if(ms)
		{//若不允许修改原矩阵
			ms->array[i] = k * m->array[i];
			return ms;
		}
		else
			m->array[i] *= k;
	}
	res = ms ? ms : m;
	return res;
}

//矩阵代数和
Martix_t* Martix_Add(float sign1, Martix_t* ml, float sign2, Martix_t* mr, Martix_t* ms)
{
	assert_param(ml->r == mr->r && ml->c == mr->c);
	for(UINT i = 0; i < ml->r * ml->c; ++i)
		ms->array[i] = sign1 * ml->array[i] + sign2 * mr->array[i];
	return ms;
}

//矩阵乘法
Martix_t* Martix_Mult(Martix_t* ml, Martix_t* mr, Martix_t* ms)
{
	assert_param(ml->c == mr->r && ms->r == ml->r && ms->c == mr->c);
	UINT row, column, mid;
	row = ml->r;
	column = mr->c;
	mid = ml->c;
	for(UINT i = 0; i < row; i++)
	{
		for(UINT j = 0; j < column; j++)
		{
			float temp = 0;
			for(UINT k = 0; k < mid; k++)
				temp += ml->array[i * mid + k] * mr->array[k * column + j];
			ms->array[i * column + j] = temp;
		}
	}
	return ms;
}

//矩阵打印
void Martix_print(Martix_t* m)
{
	assert_param(m);
	for (UINT i = 0; i < m->r; ++i)
	{
		printf("|");
		for (UINT j = 0; j < m->c; ++j)
			printf(" %10.4f ", m->array[i * m->c + j]);  //可自行修改矩阵元素打印格式
		printf("|\n");
	}
	printf("\n\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//一维数组形式的矩阵整体赋值
void aMartix_Cover(float* mat, float* data, UINT m, UINT n)
{
	assert_param(mat && data && m && n);
	for(UINT i = 0, k = 0; i < m; ++i)
		for(UINT j = 0; j < n; ++j)
			mat[i * n + j] = data[k++];
}

//一维数组形式的矩阵代数和
void aMartix_Add(float  sign1, float* ml, float sign2, float* mr, float* ms, UINT m, UINT n)
{
	assert_param(ml && mr && ms);
	for(UINT i = 0; i < m * n; ++i)
		ms[i] = sign1 * ml[i] + sign2 * mr[i];
}

//一维数组形式的矩阵数乘
void aMartix_nMul(float kp, float* mr, float* ms, UINT m, UINT n)
{
	assert_param(mr && ms);
	for(UINT i = 0; i < m * n; ++i)
		ms[i] = kp * mr[i];
}

//一维数组形式的矩阵乘法 Ms(mxp) = Ml(mxn) Mr(nxp)
void aMartix_Mul(float* ml, float* mr, float* ms, UINT m, UINT n, UINT p)
{
	assert_param(ml && mr && ms);
	for(UINT i = 0; i < m; i++)
	{
		for(UINT j = 0; j < p; j++)
		{
			float temp = 0;
			for(UINT k = 0; k < n; k++)
				temp += ml[i * n + k] * mr[k * p + j];
			ms[i * p + j] = temp;
		}
	}
}

//一维数组形式的矩阵的转置
void aMartix_Trans(float* mat, float* ms, UINT m, UINT n)
{
	assert_param(mat && ms && m && n);
	for(UINT i = 0; i < n; i++)
		for(UINT j = 0; j < m; j++)
			ms[i * m + j] = mat[j * n + i];      //对于m*n的矩阵,i*m+j表示i行j列的元素; 对于n*m的矩阵,i*n+j表示i行j列的元素
}

//一维数组形式的矩阵的行列式
//根据高斯消元法求得
float aMartix_Det(float* mat, UINT n)
{
	int i, j, k, count = 0, flag;
	float sum;
	float** res = (float**)malloc(n * sizeof(float*));
	assert_param(res && n);
	for(i = 0; i < n; i++)
	{
		res[i] = (float*)malloc(sizeof(float) * n);
		memcpy(res[i], &mat[i*n], sizeof(float) * n);//初始化
	}
	//找主元，绝对值最大的那一行，与主元行互换
	for(j = 0; j < n; j++)
	{
		flag = j;
		float Max = fabs(res[flag][j]);//用绝对值比较
		//默认当前主元行的数最大,从主对角线下方选择主元
		for(i = j; i < n; i++)
		{
			if(fabs(res[i][j]) > Max)
			{
				flag = i;
				Max = fabs(res[i][j]);
			}
		}
		if(j != flag)
		{
			count++;//记录互换次数
			float* temp = res[flag];
			res[flag] = res[j];
			res[j] = temp;
		}
		//将主对角下方元素消成0
		for(i = j + 1; i < n; i++)
		{
			float b = res[i][j] / res[j][j];
			for (k = 0; k < n; k++)
				res[i][k] -= (b * res[j][k]);
		}
	}
	//计算主对角线元素乘积
	sum = 1;
	for(i = 0, j = 0; i < n; i++, j++)
		sum *= res[i][j];
	
	for(i = 0; i < n; i++)
	{
		free(res[i]);
	}
	free(res);
	return count%2==0 ? sum : -sum;
}

//一维数组形式的矩阵的高斯法求逆，会修改原矩阵
int aMartix_Inv(float *a, UINT n)
{
    int ORDER = 10;

    signed char i,j,k,l,u,v;
    signed char is[ORDER];
    signed char js[ORDER];
    float d,p;

    for(k=0; k<=n-1; k++)
	{
        d=0.0;
        for (i=k; i<=n-1; i++) {
            for (j=k; j<=n-1; j++) {
                l=i*n+j;
                p=fabs(a[l]);
                if (p>d) {
                    d=p;
                    is[k]=i;
                    js[k]=j;
                }
            }
        }
        if (d+1.0f==1.0f) {
            return(0);
        }
        if (is[k]!=k) {
            for (j=0; j<=n-1; j++) {
                u=k*n+j;
                v=is[k]*n+j;
                p=a[u];
                a[u]=a[v];
                a[v]=p;
            }
        }
        if (js[k]!=k) {
            for (i=0; i<=n-1; i++) {
                u=i*n+k;
                v=i*n+js[k];
                p=a[u];
                a[u]=a[v];
                a[v]=p;
            }
        }
        l=k*n+k;
        a[l]=1.0f/a[l];
        for (j=0; j<=n-1; j++) {
            if (j!=k) {
                u=k*n+j;
                a[u]=a[u]*a[l];
            }
        }
        for (i=0; i<=n-1; i++) {
            if (i!=k) {
                for (j=0; j<=n-1; j++) {
                    if (j!=k) {
                        u=i*n+j;
                        a[u]=a[u]-a[i*n+k]*a[k*n+j];
                    }
                }
            }
        }
        for (i=0; i<=n-1; i++) {
            if (i!=k) {
                u=i*n+k;
                a[u]=-a[u]*a[l];
            }
        }
    }
    
    for (k=n-1; k>=0; k--) {
        if (js[k]!=k) {
            for (j=0; j<=n-1; j++)
            {   u=k*n+j;
                v=js[k]*n+j;
                p=a[u];
                a[u]=a[v];
                a[v]=p;
            }
        }
        if (is[k]!=k) {
            for (i=0; i<=n-1; i++) {
                u=i*n+k;
                v=i*n+is[k];
                p=a[u];
                a[u]=a[v];
                a[v]=p;
            }
        }
    }
    return(1);
}

//计算A*B*A'
void aMartix_AXApie(float* a, float* b, float* c, UINT m, UINT n)
{
	float Temp1[100] = {0};
	float Temp2[100] = {0};

	aMartix_Mul(a, b, Temp1, m, n, n);
	aMartix_Trans(a, Temp2, m, n);
	aMartix_Mul(Temp1, Temp2, c, n, n, m);
}

void aMartix_print(float* mat, UINT m, UINT n)
{
	for(int i = 0; i < m; i++)
	{
		printf("|");
		for(int j = 0; j < n; j++)
			printf(" %4.4f ", mat[i * n + j]);
		printf("|\n");
	}
	printf("\n");
}
