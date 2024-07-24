//
// Created by RM UI Designer
//

#include "ui_default_group3_0.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "prot_power.h"
#include "prot_judge.h"
#include "math_lib.h"
#include "arm_math.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group3_0;
ui_interface_line_t *ui_default_group3_power2 = (ui_interface_line_t *)&(ui_default_group3_0.data[0]);
ui_interface_arc_t *ui_default_group3_position2 = (ui_interface_arc_t *)&(ui_default_group3_0.data[1]);
ui_interface_number_t *ui_default_group3_time11 = (ui_interface_number_t *)&(ui_default_group3_0.data[2]);
ui_interface_number_t *ui_default_group3_time21 = (ui_interface_number_t *)&(ui_default_group3_0.data[3]);
ui_interface_number_t *ui_default_group3_time31 = (ui_interface_number_t *)&(ui_default_group3_0.data[4]);
ui_interface_number_t *ui_default_group3_time41 = (ui_interface_number_t *)&(ui_default_group3_0.data[5]);
ui_interface_number_t *ui_default_group3_time51 = (ui_interface_number_t *)&(ui_default_group3_0.data[6]);

float armour_center;
float start_angle;
float end_angle;

void _ui_init_default_group3_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group3_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_group3_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_group3_0.data[i].figure_name[2] = i + START_ID;
        ui_default_group3_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group3_0.data[i].operate_tpyel = 0;
    }

    ui_default_group3_power2->figure_tpye = 0;
    ui_default_group3_power2->layer = 0;
    ui_default_group3_power2->start_x = 1725;
    ui_default_group3_power2->start_y = 350;
    ui_default_group3_power2->end_x = 1725;
    ui_default_group3_power2->end_y = 400;
    ui_default_group3_power2->color = 0;
    ui_default_group3_power2->width = 50;
    
    ui_default_group3_position2->figure_tpye = 4;
    ui_default_group3_position2->layer = 0;
    ui_default_group3_position2->rx = 390;
    ui_default_group3_position2->ry = 390;
    ui_default_group3_position2->start_x = 957;
    ui_default_group3_position2->start_y = 537;
    ui_default_group3_position2->color = 0;
    ui_default_group3_position2->width = 10;
    ui_default_group3_position2->start_angle = 0;
    ui_default_group3_position2->end_angle = 30;

    ui_default_group3_time11->figure_tpye = 6;
    ui_default_group3_time11->layer = 0;
    ui_default_group3_time11->font_size = 20;
    ui_default_group3_time11->start_x = 120;
    ui_default_group3_time11->start_y = 900;
    ui_default_group3_time11->color = 1;
    ui_default_group3_time11->number = 6;
    ui_default_group3_time11->width = 2;

    ui_default_group3_time21->figure_tpye = 6;
    ui_default_group3_time21->layer = 0;
    ui_default_group3_time21->font_size = 20;
    ui_default_group3_time21->start_x = 120;
    ui_default_group3_time21->start_y = 875;
    ui_default_group3_time21->color = 1;
    ui_default_group3_time21->number = 5;
    ui_default_group3_time21->width = 2;

    ui_default_group3_time31->figure_tpye = 6;
    ui_default_group3_time31->layer = 0;
    ui_default_group3_time31->font_size = 20;
    ui_default_group3_time31->start_x = 120;
    ui_default_group3_time31->start_y = 850;
    ui_default_group3_time31->color = 1;
    ui_default_group3_time31->number = 4;
    ui_default_group3_time31->width = 2;

    ui_default_group3_time41->figure_tpye = 6;
    ui_default_group3_time41->layer = 0;
    ui_default_group3_time41->font_size = 20;
    ui_default_group3_time41->start_x = 120;
    ui_default_group3_time41->start_y = 825;
    ui_default_group3_time41->color = 1;
    ui_default_group3_time41->number = 2;
    ui_default_group3_time41->width = 2;

    ui_default_group3_time51->figure_tpye = 6;
    ui_default_group3_time51->layer = 0;
    ui_default_group3_time51->font_size = 20;
    ui_default_group3_time51->start_x = 120;
    ui_default_group3_time51->start_y = 800;
    ui_default_group3_time51->color = 1;
    ui_default_group3_time51->number = 1;
    ui_default_group3_time51->width = 2;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group3_0, sizeof(ui_default_group3_0));
}

void _ui_update_default_group3_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group3_0.data[i].operate_tpyel = 2;
    }
    
    ui_default_group3_power2->end_y = 350 + supercap.volume_percent;
 
    float yaw_err;
    yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    armour_center = yaw_err * 180.0f / 3.1415f + 180.0f;
    ui_default_group3_position2->start_angle = armour_center - 15.0f; 
    if(ui_default_group3_position2->start_angle < 0.0f)
        ui_default_group3_position2->start_angle +=360.0f;
    if(ui_default_group3_position2->start_angle > 360.0f)
        ui_default_group3_position2->start_angle -=360.0f;
    ui_default_group3_position2->end_angle = armour_center + 15.0f; 	
    if(ui_default_group3_position2->end_angle < 0.0f)
        ui_default_group3_position2->end_angle +=360.0f;
    if(ui_default_group3_position2->end_angle > 360.0f)
        ui_default_group3_position2->end_angle -=360.0f;	
    
    if (game_status.stage_remain_time > 360)
        ui_default_group3_time11->number = (game_status.stage_remain_time - 360)/60;
    else
        ui_default_group3_time11->number = 0;

    if (game_status.stage_remain_time > 270)
        ui_default_group3_time21->number = (game_status.stage_remain_time - 270)/60;
    else
        ui_default_group3_time21->number = 0;
    
    if (game_status.stage_remain_time > 180)
        ui_default_group3_time31->number = (game_status.stage_remain_time - 180)/60;
    else
        ui_default_group3_time31->number = 0;
    
    if (game_status.stage_remain_time > 105)
        ui_default_group3_time41->number = (game_status.stage_remain_time - 105)/60;
    else
        ui_default_group3_time41->number = 0;
    
    if (game_status.stage_remain_time > 30)
        ui_default_group3_time51->number = (game_status.stage_remain_time - 30)/60;
    else
        ui_default_group3_time51->number = 0;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group3_0, sizeof(ui_default_group3_0));
}

void _ui_remove_default_group3_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group3_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group3_0, sizeof(ui_default_group3_0));
}
