//
// Created by RM UI Designer
//

#include "ui_default_group4_0.h"

#define FRAME_ID 0
#define GROUP_ID 3
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group4_0;
ui_interface_number_t *ui_default_group4_time12 = (ui_interface_number_t *)&(ui_default_group4_0.data[0]);
ui_interface_number_t *ui_default_group4_time22 = (ui_interface_number_t *)&(ui_default_group4_0.data[1]);
ui_interface_number_t *ui_default_group4_time32 = (ui_interface_number_t *)&(ui_default_group4_0.data[2]);
ui_interface_number_t *ui_default_group4_time42 = (ui_interface_number_t *)&(ui_default_group4_0.data[3]);
ui_interface_number_t *ui_default_group4_time52 = (ui_interface_number_t *)&(ui_default_group4_0.data[4]);

void _ui_init_default_group4_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group4_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_group4_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_group4_0.data[i].figure_name[2] = i + START_ID;
        ui_default_group4_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group4_0.data[i].operate_tpyel = 0;
    }

    ui_default_group4_time12->figure_tpye = 6;
    ui_default_group4_time12->layer = 0;
    ui_default_group4_time12->font_size = 20;
    ui_default_group4_time12->start_x = 170;
    ui_default_group4_time12->start_y = 900;
    ui_default_group4_time12->color = 1;
    ui_default_group4_time12->number = 30;
    ui_default_group4_time12->width = 2;

    ui_default_group4_time22->figure_tpye = 6;
    ui_default_group4_time22->layer = 0;
    ui_default_group4_time22->font_size = 20;
    ui_default_group4_time22->start_x = 170;
    ui_default_group4_time22->start_y = 875;
    ui_default_group4_time22->color = 1;
    ui_default_group4_time22->number = 15;
    ui_default_group4_time22->width = 2;

    ui_default_group4_time32->figure_tpye = 6;
    ui_default_group4_time32->layer = 0;
    ui_default_group4_time32->font_size = 20;
    ui_default_group4_time32->start_x = 170;
    ui_default_group4_time32->start_y = 850;
    ui_default_group4_time32->color = 1;
    ui_default_group4_time32->number = 0;
    ui_default_group4_time32->width = 2;

    ui_default_group4_time42->figure_tpye = 6;
    ui_default_group4_time42->layer = 0;
    ui_default_group4_time42->font_size = 20;
    ui_default_group4_time42->start_x = 170;
    ui_default_group4_time42->start_y = 825;
    ui_default_group4_time42->color = 1;
    ui_default_group4_time42->number = 30;
    ui_default_group4_time42->width = 2;

    ui_default_group4_time52->figure_tpye = 6;
    ui_default_group4_time52->layer = 0;
    ui_default_group4_time52->font_size = 20;
    ui_default_group4_time52->start_x = 170;
    ui_default_group4_time52->start_y = 800;
    ui_default_group4_time52->color = 1;
    ui_default_group4_time52->number = 0;
    ui_default_group4_time52->width = 2;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group4_0, sizeof(ui_default_group4_0));
}

void _ui_update_default_group4_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group4_0.data[i].operate_tpyel = 2;
    }
    
    if (game_status.stage_remain_time > 360)
        ui_default_group4_time12->number = (game_status.stage_remain_time - 360)%60;
    else
        ui_default_group4_time12->number = 0;

    if (game_status.stage_remain_time > 270)
        ui_default_group4_time22->number = (game_status.stage_remain_time - 270)%60;
    else
        ui_default_group4_time22->number = 0;
    
    if (game_status.stage_remain_time > 180)
        ui_default_group4_time32->number = (game_status.stage_remain_time - 180)%60;
    else
        ui_default_group4_time32->number = 0;
    
    if (game_status.stage_remain_time > 105)
        ui_default_group4_time42->number = (game_status.stage_remain_time - 105)%60;
    else
        ui_default_group4_time42->number = 0;
    
    if (game_status.stage_remain_time > 30)
        ui_default_group4_time52->number = (game_status.stage_remain_time - 30)%60;
    else
        ui_default_group4_time52->number = 0;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group4_0, sizeof(ui_default_group4_0));
}

void _ui_remove_default_group4_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group4_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group4_0, sizeof(ui_default_group4_0));
}
