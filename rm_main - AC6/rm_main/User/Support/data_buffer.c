#include "data_buffer.h"
#include "stm32h7xx.h"
#include "string.h"
#include "stdlib.h"

/*
 * @brief     初始化buffer实例(静态分配内存)
 * @param[in] p_buffer   : buffer实例指针
 * @param[in] p_base_addr：buffer内存单元指针
 * @param[in] unit_size  : buffer内存单元长度
 * @param[in] unit_cnt   : buffer内存单元数量
 * @retval    成功返回0，失败返回-1
 */
int buffer_init(buffer_t *p_buffer, void *p_base_addr, char unit_size, int unit_cnt)
{
    assert_param(p_buffer);
    assert_param(p_base_addr);
    assert_param(unit_size);
    assert_param(unit_cnt);
    
    osMutexDef_t mute_def = {0};
    p_buffer->mutex = osMutexCreate(&mute_def);

    if (p_buffer->mutex != NULL) {
        p_buffer->p_start_addr = (char *)p_base_addr;
        p_buffer->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
        p_buffer->unit_num = unit_cnt;
        p_buffer->used_num = 0;
        p_buffer->unit_size = unit_size;
        p_buffer->p_write_addr = (char *)p_base_addr;
        return (0);
    } else {
        return (-1);
    }
}

/*
 * @brief     创建buffer实例(动态分配内存)
 * @param[in] unit_size: buffer内存单元长度
 * @param[in] unit_num : buffer内存单元数量
 * @retval    成功返回buffer指针，失败返回NULL
 */
buffer_t *buffer_create(char unit_size, int unit_cnt)
{
    buffer_t *p_buffer = NULL;
    char *p_base_addr = NULL;

    assert_param(unit_size);
    assert_param(unit_cnt);

    p_buffer = (buffer_t *)malloc(sizeof(buffer_t));
    if (p_buffer == NULL)
        return (NULL);
    p_base_addr = malloc(unit_size * unit_cnt);
    if (p_base_addr == NULL) {
        free(p_buffer);
        return (NULL);
    }
    if (buffer_init(p_buffer, p_base_addr, unit_size, unit_cnt)) {
        free(p_base_addr);
        free(p_buffer);
    }
    return (p_buffer);
}

/*
 * @brief     删除buffer实例
 * @param[in] p_buffer: buffer实例指针
 * @retval    void
 */
void buffer_destory(buffer_t *p_buffer)
{
    assert_param(p_buffer);
    assert_param(p_buffer->p_start_addr);
    free(p_buffer->p_start_addr);
    osMutexDelete(p_buffer->mutex);
    free(p_buffer);
    return;
}

/*
 * @brief     压入一个数据(加互斥锁保护)
 * @param[in] p_buffer   : buffer实例指针
 * @param[in] p_element: 压入数据指针
 * @retval    成功返回0，失败返回-1
 */
int buffer_put(buffer_t *p_buffer, const void *p_element)
{
    assert_param(p_buffer);
    assert_param(p_element);
    osMutexWait(p_buffer->mutex, osWaitForever);
    memcpy(p_buffer->p_write_addr, p_element, p_buffer->unit_size);
    p_buffer->p_write_addr += p_buffer->unit_size;
    if (p_buffer->p_write_addr > p_buffer->p_end_addr)
        p_buffer->p_write_addr = p_buffer->p_start_addr;
    p_buffer->used_num++;
    if (p_buffer->used_num > p_buffer->unit_num)
        p_buffer->used_num = p_buffer->unit_num;
    osMutexRelease(p_buffer->mutex);
    return (0);
}

/*
 * @brief     压入一个数据(不加互斥锁保护)
 * @param[in] p_buffer   : buffer实例指针
 * @param[in] p_element: 压入数据指针
 * @retval    成功返回0，失败返回-1
 */
int buffer_put_noprotect(buffer_t *p_buffer, const void *p_element)
{
    assert_param(p_buffer);
    assert_param(p_element);
    memcpy(p_buffer->p_write_addr, p_element, p_buffer->unit_size);
    p_buffer->p_write_addr += p_buffer->unit_size;
    if (p_buffer->p_write_addr > p_buffer->p_end_addr)
        p_buffer->p_write_addr = p_buffer->p_start_addr;
    p_buffer->used_num++;
    if (p_buffer->used_num > p_buffer->unit_num)
        p_buffer->used_num = p_buffer->unit_num;
    return (0);
}

/*
 * @brief     读取任意一个数据(加互斥锁保护)
 * @param[in]     p_buffer : buffer实例指针
 * @param[in]     offset   : 目标数据位置 [1, p_buffer->used_num]
 * @param[in,out] p_element: 取出数据指针
 * @retval    成功返回0，失败返回-1
 */
int buffer_get(buffer_t *p_buffer, char offset, void *p_element)
{
    char *read_addr = (void *)0;
    assert_param(p_buffer);
    assert_param(p_element);
    osMutexWait(p_buffer->mutex, osWaitForever);
    if (offset > p_buffer->used_num)
        return (-1);
    read_addr = p_buffer->p_write_addr - p_buffer->unit_size * offset;
    while (read_addr < p_buffer->p_start_addr)
        read_addr = read_addr + (p_buffer->p_end_addr - p_buffer->p_start_addr + 1);
    memcpy(p_element, read_addr, p_buffer->unit_size);
    osMutexRelease(p_buffer->mutex);
    return (0);
}

/*
 * @brief     读取任意一个数据(不加互斥锁保护)
 * @param[in]     p_buffer : buffer实例指针
 * @param[in]     offset   : 目标数据位置 [1, p_buffer->used_num]
 * @param[in,out] p_element: 取出数据指针
 * @retval    成功返回0，失败返回-1
 */
int buffer_get_noprotect(buffer_t *p_buffer, char offset, void *p_element)
{
    char *read_addr = (void *)0;
    assert_param(p_buffer);
    assert_param(p_element);
    if (offset > p_buffer->used_num)
        return (-1);
    read_addr = p_buffer->p_write_addr - p_buffer->unit_size * offset;
    while (read_addr < p_buffer->p_start_addr)
        read_addr = read_addr + (p_buffer->p_end_addr - p_buffer->p_start_addr + 1);
    memcpy(p_element, read_addr, p_buffer->unit_size);
    return (0);
}

/*
 * @brief     buffer已使用内存单元数量
 * @param[in] p_buffer: buffer实例指针
 * @retval    buffer已使用内存单元数量
 */
int buffer_used(buffer_t *p_buffer)
{
    assert_param(p_buffer);
    return (p_buffer->used_num);
}

/*
 * @brief     清空buffer所有内存
 * @param[in] p_buffer: buffer实例指针
 * @retval    成功返回0，失败返回-1
 */
int buffer_flush(buffer_t *p_buffer)
{
    assert_param(p_buffer);
    osMutexWait(p_buffer->mutex, osWaitForever);
    p_buffer->used_num = 0;
    p_buffer->p_write_addr = p_buffer->p_start_addr;
    osMutexRelease(p_buffer->mutex);
    return (0);
}

/*
 * @brief     实时更新平均滤波(压入数据同时滤波)
 * @param[in] p_buffer : buffer实例指针
 * @param[in] p_element: 压入数据指针
 * @param[in] num      : 滤波数量
 * @retval    返回平均滤波值
 * @note      仅适用于float数据单元的buffer
 */
float buffer_avg_filter(buffer_t* p_buffer, const void* p_element, uint32_t num)
{
    assert_param(p_buffer);
    assert_param(p_element);
    if (buffer_used(p_buffer) < num)
        num = buffer_used(p_buffer);
    float temp_res, temp_array = 0;
    for (int i = 0; i < num; i++) {
        buffer_get(p_buffer, i, &temp_array);
        temp_res += temp_array;
    }
    return temp_res / num;
}

/*
 * @brief     实时更新线性加权滤波(压入数据同时滤波)
 * @param[in] p_buffer : buffer实例指针
 * @param[in] p_element: 压入数据指针
 * @param[in] pweight  : 权重数组首地址
 * @param[in] num      : 滤波数量
 * @retval    返回线性加权滤波值
 * @note      仅适用于float数据单元的buffer
 */
float buffer_lin_filter(buffer_t* p_buffer, const void* p_element, float* p_weight, uint32_t num)
{
    assert_param(p_buffer);
    assert_param(p_element);
    if (buffer_used(p_buffer) < num)
        num = buffer_used(p_buffer);
    float temp_res, temp_array = 0;
    for (int i = 0; i < num; i++) {
        buffer_get(p_buffer, i, &temp_array);
        temp_res += (temp_array * p_weight[i]);
    }
    return temp_res / num;
}
