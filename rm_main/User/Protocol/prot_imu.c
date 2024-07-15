#include "prot_imu.h"
#include "string.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

imu_t chassis_imu, gimbal_imu;

/*
 * @brief     Taurus战队imu数据接收函数
 * @param[in] imu: 陀螺仪数据结构体
 * @param[in] data: 数据指针
 * @retval    void
 */
void imu_get_data(imu_t *imu, uint32_t id, uint8_t *data)
{
    float buffer[2];
    memcpy(buffer, data, 8);
    switch(id) {
    case IMU_PIT_ID: {
        imu->pit = 1.0f * buffer[0] * PI / 180;
        imu->wy = 1.0f * buffer[1] / 16.384f * PI / 180;
        break;
    }
    case IMU_YAW_ID: {
        imu->yaw = 1.0f * buffer[0] * PI / 180;
        imu->wz = 1.0f * buffer[1] / 16.384f * PI / 180;
        break;
    }
    case IMU_ROL_ID: {
        imu->rol = 1.0f * buffer[0] * PI / 180;
        imu->wx = 1.0f * buffer[1] / 16.384f * PI / 180;
        break;
    }
    case IMU_ACC_ID: {
        imu->ax = 1.0f * buffer[0];
        imu->az = 1.0f * buffer[1];
        break;
    }
    default:break;
    }
}
