#include "ui_task.h"
#include "cmsis_os.h"
#include "prot_judge.h"
#include "ui_default_group1_0.h"
#include "ui_default_group2_0.h"
#include "ui_default_group2_1.h"
#include "ui_default_group2_2.h"
#include "ui_default_group2_3.h"
#include "ui_default_group2_4.h"
#include "ui_default_group3_0.h"
#include "ui_default_group4_0.h"
#include "ui_default_group5_0.h"
#include "ui_default_group5_1.h"

#include "us_time.h"

us_time_t ui_time;

void ui_init(void)
{
    _ui_init_default_group1_0();
    _ui_init_default_group2_0();
    _ui_init_default_group2_1();
    _ui_init_default_group2_2();
    _ui_init_default_group2_3();
    _ui_init_default_group2_4();
    _ui_init_default_group3_0();
    _ui_init_default_group4_0();
    _ui_init_default_group5_0();
    _ui_init_default_group5_1();
}

void ui_update(void)
{
    _ui_update_default_group3_0();
    _ui_update_default_group4_0();
    _ui_update_default_group5_0();
    _ui_update_default_group5_1();
}

void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    ui_init();
    for(int i = 0; i < 30; i++) {
        thread_wake_time = osKernelSysTick();
        ui_init();
        osDelayUntil(&thread_wake_time, 1);
    }
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        us_timer_interval_test_start(&ui_time);
////        ui_init();
//        if (game_status.game_progress == 0 || game_status.game_progress == 1 || game_status.game_progress == 5) {
//            ui_init();
//        } else {
            ui_update();
//        }
        us_timer_interval_test_end(&ui_time);
        osDelayUntil(&thread_wake_time, 10);
    }
}
