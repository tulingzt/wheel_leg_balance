#ifndef __DATA_FIFO_H
#define __DATA_FIFO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

typedef struct
{
    char *p_start_addr; //fifo内存单元首地址
    char *p_end_addr;   //fifo内存单元尾地址
    int free_num;       //fifo内存单元空闲数量
    int used_num;       //fifo内存单元占用数量
    int unit_size;      //fifo内存单元大小(单位: Byte)
    char *p_read_addr;  //fifo读取单元指针
    char *p_write_addr; //fifo写入单元指针
    osMutexId mutex;    //fifo对应的互斥锁
} fifo_t;

int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt);
fifo_t *fifo_create(char unit_size, int unit_cnt);
void fifo_destory(fifo_t *p_fifo);
int fifo_put(fifo_t *p_fifo, const void *p_element);
int fifo_put_noprotect(fifo_t *p_fifo, const void *p_element);
int fifo_get(fifo_t *p_fifo, void *p_element);
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element);
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element);
int fifo_is_empty(fifo_t *p_fifo);
int fifo_is_full(fifo_t *p_fifo);
int fifo_used(fifo_t *p_fifo);
int fifo_free(fifo_t *p_fifo);
int fifo_flush(fifo_t *p_fifo);

#ifdef __cplusplus
}
#endif

#endif
