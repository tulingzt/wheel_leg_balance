#include "us_time.h"

prv_us_time_t prv_us_time;

/*
 * @brief 得到US_TIME_HTIM当前计数值
 */
static US_TIME_TYPE us_timer_get(void)
{
    return US_TIME_HTIM.Instance->CNT;
}

/*
 * @brief     计算两次间隔时间
 * @param[in] us_time: 毫秒定时器实例
 * @retval    void
 */
static void us_timer_minus(us_time_t* us_time)
{
    //时间间隔=计数一次时间*(溢出次数差值*溢出一次需要计数次数+计数值差值)
    us_time->dt = US_TIME_PRECISION * ((us_time->now_tim_cnt - us_time->last_tim_cnt) + 
                  (us_time->now_overflow_cnt - us_time->last_overflow_cnt) * (US_TIME_PERIOD + 1));
}

/*
 * @brief  启动定时器及其溢出中断
 * @retval void
 */
void us_timer_start(void)
{
    US_TIME_HTIM.Instance->ARR = (US_TIME_TYPE)US_TIME_PERIOD;
    HAL_TIM_Base_Start_IT(&US_TIME_HTIM);
}

/*
 * @brief  关闭定时器及其溢出中断
 * @retval void
 */
void us_timer_end(void)
{
    HAL_TIM_Base_Stop_IT(&US_TIME_HTIM);
}

/*
 * @brief     周期测试时间
 * @param[in] us_time: 毫秒定时器实例
 * @retval    返回与上次运行此函数所距离的时间
 */
float us_timer_period_test(us_time_t* us_time)
{
    us_time->now_tim_cnt = us_timer_get();
    us_time->now_overflow_cnt = prv_us_time.overflow_cnt;
    us_timer_minus(us_time);
    us_time->last_tim_cnt = us_time->now_tim_cnt;
    us_time->last_overflow_cnt = us_time->now_overflow_cnt;
    return us_time->dt;
}

/*
 * @brief     区间测试时间 开始
 * @param[in] us_time: 毫秒定时器实例
 * @retval    void
 */
void us_timer_interval_test_start(us_time_t* us_time)
{
    us_time->last_tim_cnt = us_timer_get();
    us_time->last_overflow_cnt = prv_us_time.overflow_cnt;
    us_time->interval_start_flag = 1;
}

/*
 * @brief     区间测试时间 结束
 * @param[in] us_time: 毫秒定时器实例
 * @retval    成功返回区间测试时间，失败返回-1
 */
float us_timer_interval_test_end(us_time_t* us_time)
{
    if(us_time->interval_start_flag)
    {
        us_time->now_tim_cnt = us_timer_get();
        us_time->now_overflow_cnt = prv_us_time.overflow_cnt;
        us_timer_minus(us_time);
        us_time->interval_start_flag = 0;
    }
    else
        us_time->dt = -1;
    return us_time->dt;
}

/*
 * @brief     us延时函数
 * @param[in] us: 需要延时的毫秒数
 * @retval    void
 * @note      但需要延时超过1ms时，请用osDelay或osDelayUntil以免堵塞任务
 */
void us_timer_delay(float us)
{
    prv_us_time.tim_cnt = us_timer_get();
    //第一次溢出需要当前计数值
    prv_us_time.predict_overflow_cnt = prv_us_time.overflow_cnt;
    if(us / US_TIME_PRECISION > US_TIME_PERIOD - prv_us_time.tim_cnt)
    {
        us -= ((US_TIME_TYPE)US_TIME_PERIOD - prv_us_time.tim_cnt + 1) * US_TIME_PRECISION;
        prv_us_time.predict_overflow_cnt++;
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION);
    }
    else
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION + prv_us_time.tim_cnt);
    //多次溢出计算次数和最后计数值
    while(us / US_TIME_PRECISION > US_TIME_PERIOD)
    {
        us -= ((US_TIME_TYPE)US_TIME_PERIOD + 1) * US_TIME_PRECISION;
        prv_us_time.predict_overflow_cnt++;
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION);
    }
    //当溢出计数和计数值达到要求时结束
    while(!(prv_us_time.tim_cnt >= prv_us_time.predict_tim_cnt && prv_us_time.overflow_cnt >= prv_us_time.predict_overflow_cnt))
    {
         prv_us_time.tim_cnt = us_timer_get();
    }
}
