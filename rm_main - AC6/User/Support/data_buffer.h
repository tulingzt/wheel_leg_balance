#ifndef __DATA_BUFFER_H
#define __DATA_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

typedef struct
{
    char *p_start_addr; //buffer内存单元首地址
    char *p_end_addr;   //buffer内存单元尾地址
    int used_num;       //buffer内存单元占用数量
    int unit_num;       //buffer内存单元数量
    int unit_size;      //buffer内存单元大小(单位: Byte)
    char *p_write_addr; //buffer写入单元指针
    osMutexId mutex;    //buffer对应的互斥锁
} buffer_t;

int buffer_init(buffer_t *p_buffer, void *p_base_addr, char unit_size, int unit_cnt);
buffer_t *buffer_create(char unit_size, int unit_cnt);
void buffer_destory(buffer_t *p_buffer);
int buffer_put(buffer_t *p_buffer, const void *p_element);
int buffer_put_noprotect(buffer_t *p_buffer, const void *p_element);
int buffer_get(buffer_t *p_buffer, char offset, void *p_element);
int buffer_get_noprotect(buffer_t *p_buffer, char offset, void *p_element);
int buffer_used(buffer_t *p_buffer);
int buffer_flush(buffer_t *p_buffer);
//这两个滤滤器只适用于float缓存单元的buffer
float buffer_avg_filter(buffer_t* p_buffer, const void* p_element, uint32_t num);
float buffer_lin_filter(buffer_t* p_buffer, const void* p_element, float* p_weight, uint32_t num);

#ifdef __cplusplus
}
#endif

#endif
