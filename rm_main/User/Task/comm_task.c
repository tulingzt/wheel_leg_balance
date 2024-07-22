#include "comm_task.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"

void comm_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        taskENTER_CRITICAL();
        dji_motor_output_data();
        ht_motor_output_single_data(&joint_motor[0]);
        ht_motor_output_single_data(&joint_motor[1]);
        vision_output_data();
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
        taskENTER_CRITICAL();
        ht_motor_output_single_data(&joint_motor[2]);
        ht_motor_output_single_data(&joint_motor[3]);
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
        
//        taskENTER_CRITICAL();
//        dji_motor_output_data();
//        ht_motor_output_data();
//        vision_output_data();
//        power_output_data();
//        taskEXIT_CRITICAL();
//        osDelayUntil(&thread_wake_time, 2);
    }
}
