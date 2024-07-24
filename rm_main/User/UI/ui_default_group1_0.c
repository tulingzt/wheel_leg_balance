//
// Created by RM UI Designer
//

#include "ui_default_group1_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group1_0;
ui_interface_line_t *ui_default_group1_borad1 = (ui_interface_line_t *)&(ui_default_group1_0.data[0]);
ui_interface_line_t *ui_default_group1_borad2 = (ui_interface_line_t *)&(ui_default_group1_0.data[1]);
ui_interface_line_t *ui_default_group1_borad3 = (ui_interface_line_t *)&(ui_default_group1_0.data[2]);
ui_interface_line_t *ui_default_group1_borad4 = (ui_interface_line_t *)&(ui_default_group1_0.data[3]);
ui_interface_line_t *ui_default_group1_borad5 = (ui_interface_line_t *)&(ui_default_group1_0.data[4]);
ui_interface_rect_t *ui_default_group1_power1 = (ui_interface_rect_t *)&(ui_default_group1_0.data[5]);
ui_interface_round_t *ui_default_group1_position1 = (ui_interface_round_t *)&(ui_default_group1_0.data[6]);

void _ui_init_default_group1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_group1_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_group1_0.data[i].figure_name[2] = i + START_ID;
        ui_default_group1_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group1_0.data[i].operate_tpyel = 0;
    }

    ui_default_group1_borad1->figure_tpye = 0;
    ui_default_group1_borad1->layer = 0;
    ui_default_group1_borad1->start_x = 500;
    ui_default_group1_borad1->start_y = 0;
    ui_default_group1_borad1->end_x = 720;
    ui_default_group1_borad1->end_y = 400;
    ui_default_group1_borad1->color = 1;
    ui_default_group1_borad1->width = 2;

    ui_default_group1_borad2->figure_tpye = 0;
    ui_default_group1_borad2->layer = 0;
    ui_default_group1_borad2->start_x = 1410;
    ui_default_group1_borad2->start_y = 0;
    ui_default_group1_borad2->end_x = 1215;
    ui_default_group1_borad2->end_y = 400;
    ui_default_group1_borad2->color = 1;
    ui_default_group1_borad2->width = 2;

    ui_default_group1_borad3->figure_tpye = 0;
    ui_default_group1_borad3->layer = 0;
    ui_default_group1_borad3->start_x = 950;
    ui_default_group1_borad3->start_y = 0;
    ui_default_group1_borad3->end_x = 950;
    ui_default_group1_borad3->end_y = 400;
    ui_default_group1_borad3->color = 1;
    ui_default_group1_borad3->width = 3;
    
    ui_default_group1_borad4->figure_tpye = 0;
    ui_default_group1_borad4->layer = 0;
    ui_default_group1_borad4->start_x = 460;
    ui_default_group1_borad4->start_y = 0;
    ui_default_group1_borad4->end_x = 700;
    ui_default_group1_borad4->end_y = 400;
    ui_default_group1_borad4->color = 2;
    ui_default_group1_borad4->width = 2;
    
    ui_default_group1_borad5->figure_tpye = 0;
    ui_default_group1_borad5->layer = 0;
    ui_default_group1_borad5->start_x = 1460;
    ui_default_group1_borad5->start_y = 0;
    ui_default_group1_borad5->end_x = 1260;
    ui_default_group1_borad5->end_y = 400;
    ui_default_group1_borad5->color = 2;
    ui_default_group1_borad5->width = 2;

    ui_default_group1_power1->figure_tpye = 1;
    ui_default_group1_power1->layer = 0;
    ui_default_group1_power1->start_x = 1700;
    ui_default_group1_power1->start_y = 350;
    ui_default_group1_power1->color = 1;
    ui_default_group1_power1->width = 5;
    ui_default_group1_power1->end_x = 1750;
    ui_default_group1_power1->end_y = 450;

    ui_default_group1_position1->figure_tpye = 2;
    ui_default_group1_position1->layer = 0;
    ui_default_group1_position1->r = 50;
    ui_default_group1_position1->start_x = 1725;
    ui_default_group1_position1->start_y = 600;
    ui_default_group1_position1->color = 1;
    ui_default_group1_position1->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_0, sizeof(ui_default_group1_0));
}

void _ui_update_default_group1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_0, sizeof(ui_default_group1_0));
}

void _ui_remove_default_group1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_0, sizeof(ui_default_group1_0));
}
