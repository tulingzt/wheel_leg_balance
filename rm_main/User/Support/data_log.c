#include "data_log.h"
#include "stdarg.h"

UART_HandleTypeDef *log_huart;

/*********************************** printf重定向 *********************************/
/* printf重定向使用的为HAL_UART_Transmit */
/* 取消ARM的半主机工作模式 */
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};

FILE __stdout;          
void _sys_exit(int x) 
{ 
    x = x;
}

int fputc(int ch, FILE *f)
{
    while (__HAL_UART_GET_FLAG(log_huart, UART_FLAG_TC) == RESET);
    HAL_UART_Transmit(log_huart, (uint8_t*)&ch, 1, 0xFF);
    return ch;
}

/*********************************** Log_printf实现 *********************************/
char *LOG_LEVEL_TAGS[6] = {"NULL", "ASSERT", "ERROR", "WARNING", "INFO", "DEBUG"};
char log_str[DATA_LOG_LEN];

void log_init(UART_HandleTypeDef *huart)
{
    log_huart = huart;
}

void log_printf(const char *format, ...)
{
    uint16_t len = 0;
    va_list args;
    va_start(args, format);
    //如果串口对应的DMA还未发送完就等待，防止变量UartTxBuf被CPU和DMA同时使用。
    while (log_huart->gState != HAL_UART_STATE_READY);
    len = vsnprintf((char*)log_str, sizeof(log_str)+1, (char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(log_huart, (uint8_t*)log_str, len);
}

int log_printf_to_buffer(char *buff, int size, char *format, ...)
{
    int len = 0;
    va_list arg;
    va_start(arg, format);
    len += vsnprintf(buff, size, format, arg);
    va_end(arg);
    return len;
}

/*********************************** DataScope *********************************/
/* 示波器数据结构体 */
static struct _data_scope_t
{
    unsigned char output_buffer[4 * DATA_MAX_NUM + 4]; //串口发送缓冲区
    unsigned char send_count;                          //串口需要发送的数据个数
    unsigned char data_num;                            //变量数量
} data_scope;
/*
 * @brief      将单精度浮点数据转成4字节数据并存入指定地址
 * @param[in]  target: 目标浮点数据
 * @param[out] buf   : 目标地址
 * @param[in]  offset: 地址偏置
 * @retval     void
 */
#if (DATA_LOG_MODE == 2U) || (DATA_LOG_MODE == 3U)
static void float2byte(float *target, unsigned char *buf, unsigned char offset)
{
    unsigned char *point;
    point = (unsigned char*)target;//得到float的地址
    buf[offset]   = point[0];
    buf[offset+1] = point[1];
    buf[offset+2] = point[2];
    buf[offset+3] = point[3];
}
#endif

/*
 * @brief     生成能正确识别的帧格式
 * @param[in] Channel_Number: 需要发送的通道个数
 * @retval    返回发送缓冲区数据个数
 */
static unsigned char log_scope_data_generate(unsigned char Channel_Number)
{
    if(Channel_Number == 0) {
        return 0;
    } else {
#if (DATA_LOG_MODE == 2U)//VOFA+
        uint8_t temp_cnt = Channel_Number * 4 + 4;
        data_scope.output_buffer[4 * Channel_Number + 0] = 0x00;
        data_scope.output_buffer[4 * Channel_Number + 1] = 0x00;
        data_scope.output_buffer[4 * Channel_Number + 2] = 0x80;
        data_scope.output_buffer[4 * Channel_Number + 3] = 0x7f;
        return temp_cnt;//返回一个数据包的字节数
#elif (DATA_DEBUG_MODE == 3U)//MINIBALANCE
        data_scope.output_buffer[0] = '$';//帧头
        uint8_t temp_cnt = Channel_Number * 4 + 1;
        data_scope.output_buffer[temp_cnt]  =  temp_cnt;//帧尾
        return (temp_cnt+1);//返回一个数据包的字节数
#else
        return 0;
#endif
    }
}

/*
 * @brief     将待发送通道的单精度浮点数据写入发送缓冲区
 * @param[in] Data: 目标浮点数据
 * @retval    void
 */
void log_scope_get_data(float data)
{
#if (DATA_LOG_MODE == 2U)//VOFA+
    if(data_scope.data_num >= DATA_MAX_NUM) {
        return;
    } else {
        data_scope.data_num++;
        float2byte(&data, data_scope.output_buffer, ((data_scope.data_num - 1) * 4));
    }
#elif (DATA_DEBUG_MODE == 3U)//MINIBALANCE
    if(data_scope.data_num >= 10) {//MINIBALANCE最多10个通道
        return;
    } else {
        data_scope.data_num++;
        float2byte(&data, data_scope.output_buffer, ((data_scope.data_num - 1) * 4 + 1));//留出帧头
    }
#else
    return;
#endif
}

/*
 * @brief  重写此函数，用来注册需要发送的数据
 * @retval void
 */
__weak void log_scope_data_pkg(void)
{
    //注册格式如下
    //log_scope_get_data(float_type_data1);
    //log_scope_get_data(float_type_data2);
}

/*
 * @brief  上位机通过串口打印数据波形
 * @retval void
 * @note   周期调用此函数
 */
void log_scope_data_output(void)
{
    log_scope_data_pkg();
    data_scope.send_count = log_scope_data_generate(data_scope.data_num);
    if(data_scope.send_count != 0)
        HAL_UART_Transmit_DMA(log_huart, data_scope.output_buffer, data_scope.send_count);
    data_scope.data_num = 0;
    data_scope.send_count = 0;
}
