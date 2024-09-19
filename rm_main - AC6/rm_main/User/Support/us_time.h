//模块使用的定时器，需要通过设置预分频系数，使得其固定时间改变一次
//htim2时钟频率为275M，预分频值为10，计数加一的时间为1/275M*(10+1)=4*10^(-8)=0.04us
//使用方法：
//  1. 在溢出中断中放入如下代码，修改成相应的定时器
//      if (htim->Instance == TIM5)
//      {
//          prv_us_time.overflow_cnt++;
//      }
//  2. 在 main() 中调用 ust_tim_start(); 使能定时器及其中断
//  3. 在需要计时/延时的文件中声明变量，如： ust_t ust_user;
//  4. 计时
//      区间计时：用于统计某段代码的执行时间
//          usTime_Interval_Test_Start(&ust_user);
//              ......
//          usTime_Interval_Test_End(&ust_user);
//      周期计时：用于统计某处代码的执行周期
//          usTime_Period_Test(&ust_user);
//              ......
//  5. 查看计时结果 ust_user.dt
//  6. 如使用完定时器，可以使用 usTime_End(); 失能定时器
#ifndef __US_TIME_H
#define __US_TIME_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"
#include "tim.h"
#include "stdint.h"

//模块参数设置
#define US_TIME_PRECISION   0.04f   //定时器精度，单位：us 计数值加一的一次时间
#define US_TIME_HTIM        htim2   //使用的定时器句柄 stm32h7的tim2是32位
#define US_TIME_CNT_SIZE    32      //定时器ARR寄存器位数,可通过查芯片手册时钟树获得

#if (US_TIME_CNT_SIZE == 32)
    #define US_TIME_PERIOD 0xffffffff       //定时器溢出周期数值
    #define US_TIME_TYPE uint32_t
#elif (US_TIME_CNT_SIZE == 16)
    #define US_TIME_PERIOD 0xffff           //定时器溢出周期数值
    #define US_TIME_TYPE uint16_t
#endif

typedef struct
{
    US_TIME_TYPE last_tim_cnt, now_tim_cnt;           //定时器计数值记录
    US_TIME_TYPE last_overflow_cnt, now_overflow_cnt; //定时器溢出次数记录
    float dt;                                         //单位：us
    uint8_t interval_start_flag;                      //区间测时启动标志
} us_time_t;

typedef struct
{
    __IO US_TIME_TYPE tim_cnt;
    __IO US_TIME_TYPE overflow_cnt;         //当前定时器溢出次数
    __IO US_TIME_TYPE predict_overflow_cnt; //预测延时结束时定时器溢出次数
    __IO US_TIME_TYPE predict_tim_cnt;      //预测延时结束时定时器的计数值
} prv_us_time_t;

extern prv_us_time_t prv_us_time;

void  us_timer_start(void);
void  us_timer_end(void);
float us_timer_period_test(us_time_t* us_time);
void  us_timer_interval_test_start(us_time_t* us_time);
float us_timer_interval_test_end(us_time_t* us_time);
void  us_timer_delay(float us);

#ifdef __cplusplus
}
#endif

#endif
