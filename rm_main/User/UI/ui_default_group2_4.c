//
// Created by RM UI Designer
//

#include "ui_default_group2_4.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 2
#define START_ID 4

ui_string_frame_t ui_default_group2_4;

ui_interface_string_t* ui_default_group2_text5 = &ui_default_group2_4.option;

void _ui_init_default_group2_4(void) {
    ui_default_group2_4.option.figure_name[0] = FRAME_ID;
    ui_default_group2_4.option.figure_name[1] = GROUP_ID;
    ui_default_group2_4.option.figure_name[2] = START_ID;
    ui_default_group2_4.option.operate_tpyel = 1;
    ui_default_group2_4.option.figure_tpye = 7;
    ui_default_group2_4.option.layer = 0;
    ui_default_group2_4.option.font_size = 25;
    ui_default_group2_4.option.start_x = 150;
    ui_default_group2_4.option.start_y = 805;
    ui_default_group2_4.option.color = 1;
    ui_default_group2_4.option.str_length = 1;
    ui_default_group2_4.option.width = 2;
    strcpy(ui_default_group2_text5->string, ":");

    ui_proc_string_frame(&ui_default_group2_4);
    SEND_MESSAGE((uint8_t *) &ui_default_group2_4, sizeof(ui_default_group2_4));
}

void _ui_update_default_group2_4(void) {
    ui_default_group2_4.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_group2_4);
    SEND_MESSAGE((uint8_t *) &ui_default_group2_4, sizeof(ui_default_group2_4));
}

void _ui_remove_default_group2_4(void) {
    ui_default_group2_4.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_group2_4);
    SEND_MESSAGE((uint8_t *) &ui_default_group2_4, sizeof(ui_default_group2_4));
}
