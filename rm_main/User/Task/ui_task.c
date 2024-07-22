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
}

void ui_update(void)
{
    _ui_update_default_group3_0();
    _ui_update_default_group4_0();
}

void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    ui_init();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
//        ui_init();
        if (game_status.game_progress == 0 || game_status.game_progress == 1 || game_status.game_progress == 5) {
            ui_init();
        } else {
            ui_update();
        }
        osDelayUntil(&thread_wake_time, 1);
    }
}
