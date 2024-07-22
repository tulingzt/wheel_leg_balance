#include "prot_vision.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "prot_judge.h"
#include "prot_imu.h"
#include "math_lib.h"
#include "usbd_cdc_if.h"
#include "string.h"

vision_t vision;

void vision_get_data(uint8_t *data)
{
    static vision_aim_status_e last_aim_status;
    memcpy(vision.rx[0].buff, data, VISION_DATA_LEN);
    if (vision.rx[0].data.eof != 0xCC) {
        vision.rx_status = TAIL_ERROR;
        return;
    } else if (vision.rx[0].data.cnt == vision.rx[1].data.cnt) {
        if (++vision.rx_repeat_cnt > 10) {
            vision.rx_status = REPEAT_ERROR;
            return;
        }
    }
    vision.online = 1;
    vision.rx_status = NORMAL;
    vision.rx_repeat_cnt = 0;
    vision.rx[1].data.cnt = vision.rx[0].data.cnt;
    
    NAN_PROCESS(vision.rx[0].data.yaw, vision.rx[1].data.yaw);
    NAN_PROCESS(vision.rx[0].data.pit, vision.rx[1].data.pit);
    NAN_PROCESS(vision.rx[0].data.dis, vision.rx[1].data.dis);
    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
    NAN_PROCESS(vision.rx[0].data.pos, vision.rx[1].data.pos);
    
    if (vision.rx[0].data.aim_flag) {
        vision.aim_status = AIMING;
        vision.new_frame_flag = 1;
        vision.target_yaw_angle = vision.rx[0].data.yaw / 180.0f * PI;
        vision.target_pit_angle = -vision.rx[0].data.pit / 180.0f * PI;
        vision.min_err = vision.rx[0].data.dis / 180.0f * PI;
        if (ABS(gimbal.yaw_angle.fdb - vision.target_yaw_angle) < vision.min_err && \
            ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.min_err && \
            vision.rx[0].data.fire != 0)
            vision.shoot_enable = 1;
        else
            vision.shoot_enable = 0;
    } else {
        vision.aim_status = UNAIMING;
        vision.shoot_enable = 0;
    }

    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
        vision.aim_status = FIRST_LOST;

    last_aim_status = vision.aim_status;
}

void vision_output_data(void)
{
    vision.tx.data.imu_pit = -gimbal_imu.pit / PI * 180;
    vision.tx.data.imu_yaw = gimbal_imu.yaw / PI * 180;
    vision.tx.data.imu_pit_spd = -gimbal_imu.wy / PI * 180;
    vision.tx.data.imu_yaw_spd = gimbal_imu.wz / PI * 180;
    
//    switch (shoot.shoot_speed_ref) {
//        case 15: vision.tx.data.shooter_speed = 1;break;
//        case 18: vision.tx.data.shooter_speed = 2;break;
//        case 30: vision.tx.data.shooter_speed = 3;break;
//        default: vision.tx.data.shooter_speed = 3;break;
//    }
    vision.tx.data.shooter_speed = 3;
    vision.tx.data.vacancy = 0;
    
    if (robot_status.robot_id > 100) {
        vision.tx.data.camp = 1;
    } else {
        vision.tx.data.camp = 0;
    }
    
    vision.tx.data.sof = 0x11;
    vision.tx.data.eof1 = 0x22;
    vision.tx.data.eof2 = 0x33;
    
    CDC_Transmit_HS(vision.tx.buff, 23);
}

uint8_t vision_check_offline(void)
{
    if (vision.online == 0) {
        return 1;
    } else {
        vision.online = 0;
        return 0;
    }
}
