#include "data_fifo.h"
#include "stm32h7xx.h"
#include "string.h"
#include "stdlib.h"

/*
 * @brief     初始化fifo实例(静态分配内存)
 * @param[in] p_fifo     : fifo实例指针
 * @param[in] p_base_addr：fifo内存单元指针
 * @param[in] unit_size  : fifo内存单元长度
 * @param[in] unit_cnt   : fifo内存单元数量
 * @retval    成功返回0，失败返回-1
 */
int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt)
{
    assert_param(p_fifo);
    assert_param(p_base_addr);
    assert_param(unit_size);
    assert_param(unit_cnt);
    
    osMutexDef_t mute_def = {0};
    p_fifo->mutex = osMutexCreate(&mute_def);

    if (p_fifo->mutex != NULL) {
        p_fifo->p_start_addr = (char *)p_base_addr;
        p_fifo->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
        p_fifo->free_num = unit_cnt;
        p_fifo->used_num = 0;
        p_fifo->unit_size = unit_size;
        p_fifo->p_read_addr = (char *)p_base_addr;
        p_fifo->p_write_addr = (char *)p_base_addr;
        return (0);
    } else {
        return (-1);
    }
}

/*
 * @brief     创建fifo实例(动态分配内存)
 * @param[in] unit_size: fifo内存单元长度
 * @param[in] unit_num : fifo内存单元数量
 * @retval    成功返回fifo指针，失败返回NULL
 */
fifo_t *fifo_create(char unit_size, int unit_cnt)
{
    fifo_t *p_fifo = NULL;
    char *p_base_addr = NULL;

    assert_param(unit_size);
    assert_param(unit_cnt);

    p_fifo = (fifo_t *)malloc(sizeof(fifo_t));
    if (p_fifo == NULL)
        return (NULL);
    p_base_addr = malloc(unit_size * unit_cnt);
    if (p_base_addr == NULL) {
        free(p_fifo);
        return (NULL);
    }
    if (fifo_init(p_fifo, p_base_addr, unit_size, unit_cnt)) {
        free(p_base_addr);
        free(p_fifo);
    }
    return (p_fifo);
}

/*
 * @brief     删除fifo实例
 * @param[in] p_fifo: fifo实例指针
 * @retval    void
 */
void fifo_destory(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    assert_param(p_fifo->p_start_addr);
    free(p_fifo->p_start_addr);
    osMutexDelete(p_fifo->mutex);
    free(p_fifo);
    return;
}

/*
 * @brief     压入一个数据(加互斥锁保护)
 * @param[in] p_fifo   : fifo实例指针
 * @param[in] p_element: 压入数据指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_put(fifo_t *p_fifo, const void *p_element)
{
    assert_param(p_fifo);
    assert_param(p_element);
    osMutexWait(p_fifo->mutex, osWaitForever);
    if (p_fifo->free_num == 0) {
        osMutexRelease(p_fifo->mutex);
        return (-1);
    }
    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    p_fifo->free_num--;
    p_fifo->used_num++;
    osMutexRelease(p_fifo->mutex);
    return (0);
}

/*
 * @brief     压入一个数据(不加互斥锁保护)
 * @param[in] p_fifo   : fifo实例指针
 * @param[in] p_element: 压入数据指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_put_noprotect(fifo_t *p_fifo, const void *p_element)
{
    assert_param(p_fifo);
    assert_param(p_element);
    if (p_fifo->free_num == 0)
        return (-1);
    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    p_fifo->free_num--;
    p_fifo->used_num++;
    return (0);
}

/*
 * @brief     取出一个数据(加互斥锁保护)
 * @param[in]     p_fifo   : fifo实例指针
 * @param[in,out] p_element: 取出数据指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_get(fifo_t *p_fifo, void *p_element)
{
    assert_param(p_fifo);
    assert_param(p_element);
    osMutexWait(p_fifo->mutex, osWaitForever);
    if (p_fifo->used_num == 0) {
        osMutexRelease(p_fifo->mutex);
        return (-1);
    }
    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->free_num++;
    p_fifo->used_num--;
    osMutexRelease(p_fifo->mutex);
    return (0);
}

/*
 * @brief     取出一个数据(不加互斥锁保护)
 * @param[in]     p_fifo   : fifo实例指针
 * @param[in,out] p_element: 取出数据指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element)
{
    assert_param(p_fifo);
    assert_param(p_element);
    if (p_fifo->used_num == 0)
        return (-1);
    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->free_num++;
    p_fifo->used_num--;
    return (0);
}

/*
 * @brief     提前读取一个数据
 * @param[in]     p_fifo   : fifo实例指针
 * @param[in]     offset   : 提前数量
 * @param[in,out] p_element: 取出数据指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element)
{
    char *_pre_red_index = (void *)0;
    assert_param(p_fifo);
    assert_param(p_element);
    if (offset >= p_fifo->used_num)
        return (-1);
    _pre_red_index = p_fifo->p_read_addr + p_fifo->unit_size * offset;
    while (_pre_red_index > p_fifo->p_end_addr)
        _pre_red_index = _pre_red_index - (p_fifo->p_end_addr - p_fifo->p_start_addr + 1);
    memcpy(p_element, _pre_red_index, p_fifo->unit_size);
    return (0);
}

/*
 * @brief     检查fifo是否空闲
 * @param[in] p_fifo: fifo实例指针
 * @retval    空闲返回1，否则返回0
 */
int fifo_is_empty(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    return (p_fifo->used_num == 0);
}

/*
 * @brief     检查fifo是否充满
 * @param[in] p_fifo: fifo实例指针
 * @retval    充满返回1，否则返回0
 */
int fifo_is_full(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    return (0 == p_fifo->free_num);
}

/*
 * @brief     fifo已使用内存单元数量
 * @param[in] p_fifo: fifo实例指针
 * @retval    fifo已使用内存单元数量
 */
int fifo_used(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    return (p_fifo->used_num);
}

/*
 * @brief     fifo未使用内存单元数量
 * @param[in] p_fifo: fifo实例指针
 * @retval    fifo未使用内存单元数量
 */
int fifo_free(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    return (p_fifo->free_num);
}

/*
 * @brief     清空fifo所有内存
 * @param[in] p_fifo: fifo实例指针
 * @retval    成功返回0，失败返回-1
 */
int fifo_flush(fifo_t *p_fifo)
{
    assert_param(p_fifo);
    osMutexWait(p_fifo->mutex, osWaitForever);
    p_fifo->free_num = (p_fifo->p_end_addr - p_fifo->p_start_addr) / (p_fifo->unit_size);
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->p_write_addr = p_fifo->p_start_addr;
    osMutexRelease(p_fifo->mutex);
    return (0);
}
