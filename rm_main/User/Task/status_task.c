#include "status_task.h"
#include "cmsis_os.h"

#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_vision.h"

status_t status;

void status_task(void const* argument)
{
    for(;;)
    {
        status.remote = rc_check_offline();
        status.vision = vision_check_offline();
        status.judge = judge_check_offline();
        status.power = power_check_offline();
        status.imu = imu_check_offline();
        status.dji_motor = dji_motor_check_offline();
        status.ht_motor = ht_motor_check_offline();
        
        if (status.remote == 0 && status.vision == 0 && status.judge == 0 && \
            status.power == 0 && status.imu == 0 && status.dji_motor == 0 && \
            status.ht_motor == 0) {
            status.all = 0;
        } else {
            status.all = 1;
        }
        osDelay(100);
    }
}
