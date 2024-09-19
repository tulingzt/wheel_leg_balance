#include "drv_pwm.h"

void pwm_init(pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel)
{
    pwm->htim = htim;
    pwm->channel = channel;
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

void pwm_set(pwm_t *pwm, uint32_t compare)
{
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, compare);
}
