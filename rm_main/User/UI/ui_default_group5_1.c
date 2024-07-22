//
// Created by RM UI Designer
//

#include "ui_default_group5_0.h"
#include "ui_default_group5_1.h"

#define FRAME_ID 0
#define GROUP_ID 4
#define START_ID 7
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group5_1;
ui_interface_line_t *ui_default_group5_VMC_de1 = (ui_interface_line_t *)&(ui_default_group5_1.data[0]);
ui_interface_line_t *ui_default_group5_VMC_de2 = (ui_interface_line_t *)&(ui_default_group5_1.data[1]);

void _ui_init_default_group5_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_1.data[i].figure_name[0] = FRAME_ID;
        ui_default_group5_1.data[i].figure_name[1] = GROUP_ID;
        ui_default_group5_1.data[i].figure_name[2] = i + START_ID;
        ui_default_group5_1.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group5_1.data[i].operate_tpyel = 0;
    }

    ui_default_group5_VMC_de1->figure_tpye = 0;
    ui_default_group5_VMC_de1->layer = 0;
//    ui_default_group5_VMC_de1->start_x = 1490;
//    ui_default_group5_VMC_de1->start_y = 732;
//    ui_default_group5_VMC_de1->end_x = 1540;
//    ui_default_group5_VMC_de1->end_y = 782;
    ui_default_group5_VMC_de1->color = 0;
    ui_default_group5_VMC_de1->width = 1;

    ui_default_group5_VMC_de2->figure_tpye = 0;
    ui_default_group5_VMC_de2->layer = 0;
//    ui_default_group5_VMC_de2->start_x = 1497;
//    ui_default_group5_VMC_de2->start_y = 727;
//    ui_default_group5_VMC_de2->end_x = 1547;
//    ui_default_group5_VMC_de2->end_y = 777;
    ui_default_group5_VMC_de2->color = 1;
    ui_default_group5_VMC_de2->width = 1;
    
    ui_default_group5_VMC_de1->start_x = ui_default_group5_VMC_cd1->end_x;
    ui_default_group5_VMC_de1->start_y = ui_default_group5_VMC_cd1->end_y;
    ui_default_group5_VMC_de1->end_x = ui_default_group5_VMC_ae->end_x;
    ui_default_group5_VMC_de1->end_y = ui_default_group5_VMC_ae->end_y;
    
    ui_default_group5_VMC_de2->start_x = ui_default_group5_VMC_cd2->end_x;
    ui_default_group5_VMC_de2->start_y = ui_default_group5_VMC_cd2->end_y;
    ui_default_group5_VMC_de2->end_x = ui_default_group5_VMC_ae->end_x;
    ui_default_group5_VMC_de2->end_y = ui_default_group5_VMC_ae->end_y;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_1, sizeof(ui_default_group5_1));
}

void _ui_update_default_group5_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_1.data[i].operate_tpyel = 2;
    }
    
    ui_default_group5_VMC_de1->start_x = ui_default_group5_VMC_cd1->end_x;
    ui_default_group5_VMC_de1->start_y = ui_default_group5_VMC_cd1->end_y;
    ui_default_group5_VMC_de1->end_x = ui_default_group5_VMC_ae->end_x;
    ui_default_group5_VMC_de1->end_y = ui_default_group5_VMC_ae->end_y;
    
    ui_default_group5_VMC_de2->start_x = ui_default_group5_VMC_cd2->end_x;
    ui_default_group5_VMC_de2->start_y = ui_default_group5_VMC_cd2->end_y;
    ui_default_group5_VMC_de2->end_x = ui_default_group5_VMC_ae->end_x;
    ui_default_group5_VMC_de2->end_y = ui_default_group5_VMC_ae->end_y;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_1, sizeof(ui_default_group5_1));
}

void _ui_remove_default_group5_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_1.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_1, sizeof(ui_default_group5_1));
}
