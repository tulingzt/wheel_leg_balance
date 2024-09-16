#ifndef __DRV_WS2812B_H
#define __DRV_WS2812B_H

#include "stdint.h"

#define RGB_NUM 7
#define ZERO_NUM 0

typedef struct
{
    uint8_t r, g, b;
} rgb_color_t;

typedef struct
{
    uint8_t rgb_status[RGB_NUM];
    uint8_t rgb_buffer[24 * RGB_NUM + 2 * ZERO_NUM];
    rgb_color_t rgb_data[RGB_NUM];
} rgb_t;

extern rgb_t rgb;

void rgb_change(uint8_t i, uint8_t status);
void rgb_set_bright(uint8_t i, uint8_t bright);
void rgb_output_data(void);

#endif
