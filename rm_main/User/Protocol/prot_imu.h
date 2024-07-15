#ifndef __PROT_IMU_H
#define __PROT_IMU_H

#include "stm32h7xx.h"

#define IMU_PIT_ID 0x011
#define IMU_YAW_ID 0x012
#define IMU_ROL_ID 0x013
#define IMU_ACC_ID 0x014

#define IMU_PALSTANCE_ID    0x013
#define IMU_ANGLE_ID        0x014
#define IMU_ELSE_ID         0x015

typedef struct
{
    //反馈数据
    float pit, yaw, rol;    //rad 方向分别为 右 上 前(右手螺旋)
    float wy, wz, wx;       //rad/s
    float ay, az, ax;       //m/(s^2)
} imu_t;

extern imu_t chassis_imu, gimbal_imu;

void imu_get_data(imu_t *imu, uint32_t id, uint8_t *data);

#endif
