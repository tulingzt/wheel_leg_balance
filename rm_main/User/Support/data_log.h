/*使用方法：
 *  0. 选择所用串口
 *  1. 在 data_scope.h 选择使用的上位机，根据需要修改示波器通道数
 *     其中，若使用 MiniBalance，则最大通道数默认10
 *  2. 改写 __weak void DataWavePkg(void) 函数，在其中加入待发送的数据
 *     void DataWavePkg(void)
 *     {
 *          the data needed to send:
 *          DataScope_Get_Channel_Data(float_type_data1);
 *          DataScope_Get_Channel_Data(float_type_data2);
 *     }
 *  3. 周期执行 DataWave()
 *     函数即可一次性将注册数据当前值顺序发出
 */

#ifndef __DATA_LOG_H
#define __DATA_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "usart.h"
#include "stdio.h"

/*DATA_DEBUG_LEVEL为0U时，取消各种Log
 *                为1U时，使用Log打印日志
 *                为2U时，为VOFA+打印波形
 *                为3U时，为MiniBalance打印波形
 */
#define DATA_LOG_MODE   2U
#define DATA_MAX_NUM    16                    //VOFA+示波器最大通道数
#define DATA_LOG_LEN    128                   //使用DMA_printf时字符串最大长度

/* Log格式使能 */
#define LOG_TIMESTAMP_EN     1
#define LOG_FUNCTION_EN      1
#define LOG_FILE_LINE_EN     1

/* 各等级Log使能 */
#define LOG_ASSERT_EN        1
#define LOG_ERROR_EN         1
#define LOG_WARINING_EN      1
#define LOG_INFO_EN          1
#define LOG_DEBUG_EN         1

#define LOG_ASSERT   1
#define LOG_ERROR    2
#define LOG_WARINING 3
#define LOG_INFO     4
#define LOG_DEBUG    5

#if (DATA_LOG_MODE == 1U)
    #define log_p(format, ...) log_printf(format, ##__VA_ARGS__)
#else
    #define log_p(...)
#endif
                                   
#if (LOG_ASSERT_EN == 1) && (DATA_LOG_MODE == 1U)
    #define log_a(format, ...) log_level(LOG_ASSERT, format, ##__VA_ARGS__)
#else
    #define log_a(...)
#endif

#if (LOG_ERROR_EN == 1) && (DATA_LOG_MODE == 1U)
    #define log_e(format, ...) log_level(LOG_ERROR, format, ##__VA_ARGS__)
#else
    #define log_e(...)
#endif

#if (LOG_WARINING_EN == 1) && (DATA_LOG_MODE == 1U)
    #define log_w(format, ...) log_level(LOG_WARINING, format, ##__VA_ARGS__)
#else
    #define log_w(...)
#endif

#if (LOG_INFO_EN == 1) && (DATA_LOG_MODE == 1U)
    #define log_i(format, ...) log_level(LOG_INFO, format, ##__VA_ARGS__)
#else
    #define log_i(...)
#endif

#if (LOG_DEBUG_EN == 1) && (DATA_LOG_MODE == 1U)
    #define log_d(format, ...) log_level(LOG_DEBUG, format, ##__VA_ARGS__)
#else
    #define log_d(...)
#endif

#define log_level(level, ...)                                                                                                      \
    do                                                                                                                             \
    {                                                                                                                              \
        uint16_t len = 0;                                                                                                          \
        while ((&DATA_LOG_UART)->gState != HAL_UART_STATE_READY);                                                                  \
        len += snprintf(&log_str[len], DATA_LOG_LEN - len, "\r\n");                                                                \
        if(LOG_TIMESTAMP_EN)                                                                                                       \
            len += snprintf(&log_str[len], DATA_LOG_LEN - len, "[%d.%03d]", (int)HAL_GetTick() / 1000, (int)HAL_GetTick() % 1000); \
        len += snprintf(&log_str[len], DATA_LOG_LEN - len, "[%s]", LOG_LEVEL_TAGS[level]);                                         \
        if(LOG_FUNCTION_EN)                                                                                                        \
            len += snprintf(&log_str[len], DATA_LOG_LEN - len, "[%s]", __FUNCTION__);                                              \
        if((LOG_FILE_LINE_EN) && (level <= LOG_ERROR))                                                                             \
            len += snprintf(&log_str[len], DATA_LOG_LEN - len, "(%s,%d)", __FILE__, __LINE__);                                     \
        len += log_printf_to_buffer(&log_str[len], DATA_LOG_LEN - len, __VA_ARGS__);                                               \
        HAL_UART_Transmit_DMA(&DATA_LOG_UART, (uint8_t*)log_str, len);                                                             \
    } while(0)

extern char log_str[DATA_LOG_LEN];
extern char *LOG_LEVEL_TAGS[];

int fputc(int ch, FILE *f);

void log_printf(const char *format, ...);
int  log_printf_to_buffer(char *buff, int size, char *format, ...);

//示波器相关函数
void log_init(UART_HandleTypeDef *huart);
void log_scope_data_pkg(void);          //重写此弱函数
void log_scope_get_data(float Data);    //按顺序注册待打印数据
void log_scope_data_output(void);       //一次性按顺序将注册数据打印到上位机

#ifdef __cplusplus
}
#endif

#endif
