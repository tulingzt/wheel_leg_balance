#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} pwm_t;

void pwm_init(pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel);
void pwm_set(pwm_t *pwm, uint32_t compare);

#endif
