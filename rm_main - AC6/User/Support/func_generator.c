#include "func_generator.h"
#include "stdlib.h"

/*
 * @brief     正弦信号生成器初始化
 * @param[in] 各参数详细见结构体说明
 * @retval    void
 */
void FGT_sin_init(FGT_sin_t* sin, float Td, float dc, float T, float A, float phi)
{
    sin->Td = Td;
    sin->time = 0;
    sin->dc = dc;
    sin->A = A;
    sin->T = T;
    sin->phi = phi;
    sin->out = 0;
}

/*
 * @brief     产生正弦信号
 * @param[in] sin: 正弦信号生成器实例
 * @retval    返回正弦信号值
 */
float FGT_sin_calc(FGT_sin_t* sin)
{
    float ac = 0;
    ac = (sin->A) * sinf((2 * PI / sin->T) * (sin->time) + sin->phi); //计算交流量
    sin->time = fmodf(sin->time + sin->Td, sin->T);                   //迭代时间(先输出再计算，使从0开始)
    sin->out = ac + sin->dc;                                          //计算交直流量，迭代输出记录
    return sin->out;
}

/*
 * @brief     方波信号生成器初始化
 * @param[in] 各参数详细见结构体说明
 * @retval    void
 */
void FGT_sqr_init(FGT_sqr_t* sqr, float Td, float Th, float Tl, float high, float low)
{
    sqr->Td = Td;
    sqr->time = 0;
    sqr->high = high;
    sqr->low = low;
    sqr->Th = Th;
    sqr->Tl = Tl;
    sqr->out = 0;
}

/*
 * @brief     产生方波信号
 * @param[in] sqr: 方波信号生成器实例
 * @retval    返回方波信号值
 */
float FGT_sqr_calc(FGT_sqr_t* sqr)
{
    float dc = 0;
    if(sqr->time > sqr->Th)                                    //计算直流量
        dc = sqr->low;
    else
        dc = sqr->high;
    sqr->time = fmodf(sqr->time + sqr->Td, sqr->Th + sqr->Tl); //迭代时间(先输出再计算，使从0开始)
    sqr->out = dc;                                             //迭代输出记录
    return sqr->out;
}

/*
 * @brief     角波信号生成器初始化
 * @param[in] 各参数详细见结构体说明
 * @retval    void
 */
void FGT_agl_init(FGT_agl_t* agl, float Td, float T1, float T2, float T1_out, float T2_out)
{
    agl->Td = Td;
    agl->time = 0;
    agl->T1 = T1;
    agl->T2 = T2;
    agl->T1_out = T1_out;
    agl->T2_out  = T2_out;
    agl->out = 0;
}

/*
 * @brief     产生角波信号
 * @param[in] agl: 角波信号生成器实例
 * @retval    返回角波信号值
 */
float FGT_agl_calc(FGT_agl_t* agl)
{
    float ac = 0;
    //确定区间，计算相对时间，计算交流量
    if(agl->time < agl->T1) //在第一个拐点之前
        ac = ((agl->T1_out-agl->T2_out)/agl->T1) * agl->time + agl->T2_out;
    else                    //在两个拐点之间
        ac = ((agl->T2_out-agl->T1_out)/agl->T2) * (agl->time - agl->T1) + agl->T1_out;
    agl->time = fmodf(agl->time+agl->Td, agl->T1 + agl->T2); //迭代时间
    agl->out = ac;                                           //迭代输出记录
    return agl->out;
}

/*
 * @brief     符号函数信号生成器初始化
 * @param[in] 各参数详细见结构体说明
 * @retval    void
 */
void FGT_npz_init(FGT_npz_t* npz, float Td, float T1, float T2, float T3)
{
    npz->Td = Td;
    npz->time = 0;
    npz->T1 = T1;
    npz->T2 = T2;
    npz->T3 = T3;
    npz->out = 0;
}

/*
 * @brief     产生符号函数信号
 * @param[in] npz: 符号函数信号生成器实例
 * @retval    返回符号函数信号值
 */
float FGT_npz_calc(FGT_npz_t* npz)
{
    if (npz->time <= npz->T1)                                                                //在第一个拐点之前
        npz->out = -1;
    else if ((npz->T1 < npz->time) && (npz->time < npz->T1 + npz->T2))                       //在两个拐点之间
        npz->out = 0;
    else if ((npz->T1 + npz->T2 <= npz->time) && (npz->time <= npz->T1 + npz->T2 + npz->T3)) //在第二个拐点之后
        npz->out = 1;
    npz->time = fmodf(npz->time+npz->Td, npz->T1 + npz->T2 + npz->T3); //迭代时间(先输出再计算，使从0开始)
    return npz->out;
}

/*
 * @brief     一般函数信号生成器初始化
 * @param[in] 各参数详细见结构体说明
 * @retval    void
 */
void FGT_f_init(FGT_f_t* pf, float (*f)(float time), float Td, float T)
{
    pf->f = f;
    pf->T = T;
    pf->Td = Td;
    pf->time = 0;
    pf->out = 0;
}

/*
 * @brief     产生一般函数信号
 * @param[in] npz: 一般函数信号生成器实例
 * @retval    返回一般函数信号值
 */
float FGT_f_calc(FGT_f_t* pf)
{
    pf->time = fmodf(pf->time+pf->Td,pf->T); //迭代时间(先输出再计算，使从0开始)
    pf->out = pf->f(pf->time);
    return pf->out;
}

/*
 * @brief     生成[min,max]范围的随机数 运行一次函数需要1.7us左右
 * @param[in] min: 随机数最小值
 * @param[in] max: 随机数最大值
 * @retval    返回随机数
 */
float FGT_random_generate(float min, float max)
{
    uint32_t temp_32_rng;
//    while (__HAL_RNG_GET_FLAG(&hrng, RNG_FLAG_DRDY) == RESET);
    HAL_RNG_GenerateRandomNumber(&hrng, &temp_32_rng);
    return (float)temp_32_rng / 0xffffffff * (max - min) + min;
}

/*
 * @brief     生成高斯噪声
 * @param[in] mu   : 均值
 * @param[in] sigma: 标准差
 * @retval    返回高斯噪声值
 */
float FGT_gauss_generate(float mu, float sigma)
{
    float t1, t2, a, r, x;
    //产生两个均匀分布的0~1的随机序列 两种实现方式：硬件实现和软件实现
    //两个实现方式的结果差不多，但软件实现会比硬件实现少2us，因为硬件需要等待寄存器更新
    //软件实现需要2.7到3.8us，硬件实现需要4.2到5.2us
    t1 = (float)rand() / (RAND_MAX);
    t2 = (float)rand() / (RAND_MAX);
//    t1 = FGT_RandomGenerate(0, 1);
//    t2 = FGT_RandomGenerate(0, 1);
    //极坐标的两个随机变量分布序列
    a = 2 * PI * t1;            //a是极坐标的角度：变成了0~2*pi的均匀分布
    r = sqrt(-2 * logf(t2));    //r是极坐标的距离：变成自然对数开根号的一种分布
    //用极坐标(a,r)转换成笛卡尔坐标(x,y)，这就是产生的高斯白噪声
    x = r * cos(a);
    return mu + sigma * x;
}
