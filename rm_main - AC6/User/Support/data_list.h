#ifndef __DATA_LIST_H
#define __DATA_LIST_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _list_t
{
    struct _list_t *next, *prev;
} list_t;

/*
 * @brief 创建list并初始化
 */
#define list_creat(name) \
    list_t name = {&(name), &(name)}

/*
 * @brief  初始化list，置空list
 * @retval void
 */
static __inline void list_init(list_t *list)
{
    list->next = list;
    list->prev = list;
}

/*
 * @brief  在prev和next之间插入new
 * @retval void
 */
static __inline void __list_add(list_t *new, list_t *prev, list_t *next)
{
    next->prev = new;
    new->next = next;
    prev->next = new;
    new->prev = prev;
}

/*
 * @brief  在head之后插入new
 * @retval void
 */
static __inline void list_add(list_t *new, list_t *head)
{
    __list_add(new, head, head->next);
}

/*
 * @brief  将prev和next联系起来
 * @retval void
 */
static __inline void __list_del(list_t *prev, list_t *next)
{
    next->prev = prev;
    prev->next = next;
}

/*
 * @brief  在链表中去掉entry并将entry置空
 * @retval void
 */
static __inline void list_del(list_t *entry)
{
    __list_del(entry->prev, entry->next);
    list_init(entry);
}

/*
 * @brief  用new来代替old所在链表中old的位置，并将old置空
 * @retval void
 * @note   如果old为空，则new也为空
 */
static __inline void list_replace(list_t *old, list_t *new)
{
    new->next = old->next;
    new->next->prev = new;
    new->prev = old->prev;
    new->prev->next = new;
    list_init(old);
}

/*
 * @brief  将list从原来的链表移动到head后
 * @retval void
 */
static __inline void list_move(list_t *list, list_t *head)
{
    __list_del(list->prev, list->next);
    list_add(list, head);
}

/*
 * @brief     检查list是否在尾部
 * @param[in] head: list指针
 * @retval    在尾部返回1，否则返回0
 */
static __inline int list_is_last(const list_t *head)
{
    return head->next == head;
}

/*
 * @brief     检查list是否为空
 * @param[in] head: list指针
 * @retval    空闲返回1，否则返回0
 */
static __inline int list_is_empty(const list_t *head)
{
    return (head->next == head) && (head->prev == head);
}

/*
 * @brief     得到ptr所在结构体的指针
 * @param[in] ptr   :目标结构体的指针
 * @param[in] type  :结构体类型
 * @param[in] member:ptr在type中的具体成员
 * @retval    
 */
#define list_entry(ptr, type, member) \
  ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

/*
 * @brief     对列表进行遍历
 * @param[in] pos : 当前遍历所在
 * @param[in] n   : 下个遍历所在
 * @param[in] head: 被遍历的列表头
 */
#define list_for_each(pos, head) \
  for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_prev(pos, head) \
  for (pos = (head)->prev; pos != (head); pos = pos->prev)

#define list_for_each_safe(pos, n, head)  \
  for (pos = (head)->next, n = pos->next; \
       pos != (head);                     \
       pos = n, n = pos->next)

#define list_for_each_prev_safe(pos, n, head) \
  for (pos = (head)->prev, n = pos->prev;     \
       pos != (head);                         \
       pos = n, n = pos->prev)

#ifdef __cplusplus
}
#endif

#endif
