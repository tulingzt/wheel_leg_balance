//
// Created by RM UI Designer
//

#include "ui_default_group5_0.h"

#include "leg_vmc.h"
#include "wlr.h"
#include "arm_math.h"

#define FRAME_ID 0
#define GROUP_ID 4
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group5_0;
ui_interface_line_t *ui_default_group5_VMC_ae = (ui_interface_line_t *)&(ui_default_group5_0.data[0]);
ui_interface_line_t *ui_default_group5_VMC_ab1 = (ui_interface_line_t *)&(ui_default_group5_0.data[1]);
ui_interface_line_t *ui_default_group5_VMC_ab2 = (ui_interface_line_t *)&(ui_default_group5_0.data[2]);
ui_interface_line_t *ui_default_group5_VMC_bc1 = (ui_interface_line_t *)&(ui_default_group5_0.data[3]);
ui_interface_line_t *ui_default_group5_VMC_bc2 = (ui_interface_line_t *)&(ui_default_group5_0.data[4]);
ui_interface_line_t *ui_default_group5_VMC_cd1 = (ui_interface_line_t *)&(ui_default_group5_0.data[5]);
ui_interface_line_t *ui_default_group5_VMC_cd2 = (ui_interface_line_t *)&(ui_default_group5_0.data[6]);

uint32_t x0 = 1600, y0 = 700;
const float LegLength[5] = {75, 75, 135, 135, 75};

void _ui_init_default_group5_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_group5_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_group5_0.data[i].figure_name[2] = i + START_ID;
        ui_default_group5_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group5_0.data[i].operate_tpyel = 0;
    }

    ui_default_group5_VMC_ae->figure_tpye = 0;
    ui_default_group5_VMC_ae->layer = 0;
    ui_default_group5_VMC_ae->color = 0;
    ui_default_group5_VMC_ae->width = 1;

    ui_default_group5_VMC_ab1->figure_tpye = 0;
    ui_default_group5_VMC_ab1->layer = 0;
    ui_default_group5_VMC_ab1->color = 0;
    ui_default_group5_VMC_ab1->width = 1;

    ui_default_group5_VMC_ab2->figure_tpye = 0;
    ui_default_group5_VMC_ab2->layer = 0;
    ui_default_group5_VMC_ab2->color = 1;
    ui_default_group5_VMC_ab2->width = 1;

    ui_default_group5_VMC_bc1->figure_tpye = 0;
    ui_default_group5_VMC_bc1->layer = 0;
    ui_default_group5_VMC_bc1->color = 0;
    ui_default_group5_VMC_bc1->width = 1;

    ui_default_group5_VMC_bc2->figure_tpye = 0;
    ui_default_group5_VMC_bc2->layer = 0;
    ui_default_group5_VMC_bc2->color = 1;
    ui_default_group5_VMC_bc2->width = 1;

    ui_default_group5_VMC_cd1->figure_tpye = 0;
    ui_default_group5_VMC_cd1->layer = 0;
    ui_default_group5_VMC_cd1->color = 0;
    ui_default_group5_VMC_cd1->width = 1;

    ui_default_group5_VMC_cd2->figure_tpye = 0;
    ui_default_group5_VMC_cd2->layer = 0;
    ui_default_group5_VMC_cd2->color = 1;
    ui_default_group5_VMC_cd2->width = 1;
    
    ui_default_group5_VMC_ae->start_x = x0 - LegLength[0]/2*arm_cos_f32(-wlr.pit_fdb);//A
    ui_default_group5_VMC_ae->start_y = y0 + LegLength[0]/2*arm_sin_f32(-wlr.pit_fdb);
    ui_default_group5_VMC_ae->end_x = x0 + LegLength[0]/2*arm_cos_f32(-wlr.pit_fdb);//E
    ui_default_group5_VMC_ae->end_y = y0 - LegLength[0]/2*arm_sin_f32(-wlr.pit_fdb);
    
    ui_default_group5_VMC_ab1->start_x = ui_default_group5_VMC_ae->start_x;//A
    ui_default_group5_VMC_ab1->start_y = ui_default_group5_VMC_ae->start_y;
    ui_default_group5_VMC_ab1->end_x = ui_default_group5_VMC_ae->start_x + LegLength[1]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[1]);//B1
    ui_default_group5_VMC_ab1->end_y = ui_default_group5_VMC_ae->start_y - LegLength[1]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[1]);
    
    ui_default_group5_VMC_ab2->start_x = ui_default_group5_VMC_ae->start_x;//A
    ui_default_group5_VMC_ab2->start_y = ui_default_group5_VMC_ae->start_y;
    ui_default_group5_VMC_ab2->end_x = ui_default_group5_VMC_ae->start_x + LegLength[1]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[1]);//B2
    ui_default_group5_VMC_ab2->end_y = ui_default_group5_VMC_ae->start_y - LegLength[1]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[1]);
    
    ui_default_group5_VMC_bc1->start_x = ui_default_group5_VMC_ab1->end_x;//B1
    ui_default_group5_VMC_bc1->start_y = ui_default_group5_VMC_ab1->end_y;
    ui_default_group5_VMC_bc1->end_x = ui_default_group5_VMC_ab1->end_x + LegLength[2]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[2]);//C1
    ui_default_group5_VMC_bc1->end_y = ui_default_group5_VMC_ab1->end_y - LegLength[2]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[2]);
    
    ui_default_group5_VMC_bc2->start_x = ui_default_group5_VMC_ab2->end_x;//B2
    ui_default_group5_VMC_bc2->start_y = ui_default_group5_VMC_ab2->end_y;
    ui_default_group5_VMC_bc2->end_x = ui_default_group5_VMC_ab2->end_x + LegLength[2]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[2]);//C2
    ui_default_group5_VMC_bc2->end_y = ui_default_group5_VMC_ab2->end_y - LegLength[2]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[2]);
    
    ui_default_group5_VMC_cd1->start_x = ui_default_group5_VMC_bc1->end_x;//C1
    ui_default_group5_VMC_cd1->start_y = ui_default_group5_VMC_bc1->end_y;
    ui_default_group5_VMC_cd1->end_x = ui_default_group5_VMC_ae->end_x + LegLength[4]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[4]);//D1
    ui_default_group5_VMC_cd1->end_y = ui_default_group5_VMC_ae->end_y - LegLength[4]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[4]);
    
    ui_default_group5_VMC_cd2->start_x = ui_default_group5_VMC_bc2->end_x;//C2
    ui_default_group5_VMC_cd2->start_y = ui_default_group5_VMC_bc2->end_y;
    ui_default_group5_VMC_cd2->end_x = ui_default_group5_VMC_ae->end_x + LegLength[4]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[4]);//D1
    ui_default_group5_VMC_cd2->end_y = ui_default_group5_VMC_ae->end_y - LegLength[4]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[4]);


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_0, sizeof(ui_default_group5_0));
}

void _ui_update_default_group5_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_0.data[i].operate_tpyel = 2;
    }
    
    ui_default_group5_VMC_ae->start_x = x0 - LegLength[0]/2*arm_cos_f32(-wlr.pit_fdb);//A
    ui_default_group5_VMC_ae->start_y = y0 + LegLength[0]/2*arm_sin_f32(-wlr.pit_fdb);
    ui_default_group5_VMC_ae->end_x = x0 + LegLength[0]/2*arm_cos_f32(-wlr.pit_fdb);//E
    ui_default_group5_VMC_ae->end_y = y0 - LegLength[0]/2*arm_sin_f32(-wlr.pit_fdb);
    
    ui_default_group5_VMC_ab1->start_x = ui_default_group5_VMC_ae->start_x;//A
    ui_default_group5_VMC_ab1->start_y = ui_default_group5_VMC_ae->start_y;
    ui_default_group5_VMC_ab1->end_x = ui_default_group5_VMC_ae->start_x + LegLength[1]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[1]);//B1
    ui_default_group5_VMC_ab1->end_y = ui_default_group5_VMC_ae->start_y - LegLength[1]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[1]);
    
    ui_default_group5_VMC_ab2->start_x = ui_default_group5_VMC_ae->start_x;//A
    ui_default_group5_VMC_ab2->start_y = ui_default_group5_VMC_ae->start_y;
    ui_default_group5_VMC_ab2->end_x = ui_default_group5_VMC_ae->start_x + LegLength[1]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[1]);//B2
    ui_default_group5_VMC_ab2->end_y = ui_default_group5_VMC_ae->start_y - LegLength[1]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[1]);
    
    ui_default_group5_VMC_bc1->start_x = ui_default_group5_VMC_ab1->end_x;//B1
    ui_default_group5_VMC_bc1->start_y = ui_default_group5_VMC_ab1->end_y;
    ui_default_group5_VMC_bc1->end_x = ui_default_group5_VMC_ab1->end_x + LegLength[2]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[2]);//C1
    ui_default_group5_VMC_bc1->end_y = ui_default_group5_VMC_ab1->end_y - LegLength[2]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[2]);
    
    ui_default_group5_VMC_bc2->start_x = ui_default_group5_VMC_ab2->end_x;//B2
    ui_default_group5_VMC_bc2->start_y = ui_default_group5_VMC_ab2->end_y;
    ui_default_group5_VMC_bc2->end_x = ui_default_group5_VMC_ab2->end_x + LegLength[2]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[2]);//C2
    ui_default_group5_VMC_bc2->end_y = ui_default_group5_VMC_ab2->end_y - LegLength[2]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[2]);
    
    ui_default_group5_VMC_cd1->start_x = ui_default_group5_VMC_bc1->end_x;//C1
    ui_default_group5_VMC_cd1->start_y = ui_default_group5_VMC_bc1->end_y;
    ui_default_group5_VMC_cd1->end_x = ui_default_group5_VMC_ae->end_x + LegLength[4]*arm_cos_f32(-wlr.pit_fdb+vmc[0].q_fdb[4]);//D1
    ui_default_group5_VMC_cd1->end_y = ui_default_group5_VMC_ae->end_y - LegLength[4]*arm_sin_f32(-wlr.pit_fdb+vmc[0].q_fdb[4]);
    
    ui_default_group5_VMC_cd2->start_x = ui_default_group5_VMC_bc2->end_x;//C2
    ui_default_group5_VMC_cd2->start_y = ui_default_group5_VMC_bc2->end_y;
    ui_default_group5_VMC_cd2->end_x = ui_default_group5_VMC_ae->end_x + LegLength[4]*arm_cos_f32(-wlr.pit_fdb+vmc[1].q_fdb[4]);//D1
    ui_default_group5_VMC_cd2->end_y = ui_default_group5_VMC_ae->end_y - LegLength[4]*arm_sin_f32(-wlr.pit_fdb+vmc[1].q_fdb[4]);
    

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_0, sizeof(ui_default_group5_0));
}

void _ui_remove_default_group5_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group5_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group5_0, sizeof(ui_default_group5_0));
}
