#ifndef __MODE_SWITCH_TASK_H
#define __MODE_SWITCH_TASK_H

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,   //保护模式
    REMOTER_MODE,   //遥控模式
    KEYBOARD_MODE,  //键盘模式
} ctrl_mode_e;

extern uint8_t lock_flag, reset_flag;
extern ctrl_mode_e ctrl_mode;

void mode_switch_task(void const *argu);

#endif
