#include "drv_ws2812b.h"
#include "spi.h"
#include "dma.h"

#define brightness 0x05 //亮度 0x00-0xff
const uint8_t code[] = {0xC0, 0xF8};
extern DMA_HandleTypeDef hdma_spi1_tx;

rgb_t rgb = {0};

void rgb_set_color(uint8_t id, rgb_color_t color)
{
    if (id < RGB_NUM) {
        rgb.rgb_data[id].g = color.g;
        rgb.rgb_data[id].b = color.b;
        rgb.rgb_data[id].r = color.r;
    }
}

void rgb_reflash(uint8_t reflash_num)
{
    uint8_t data_b, data_r, data_g;
    for (int i = 0; i < reflash_num; i++) {
        data_g = rgb.rgb_data[i].g;
        data_r = rgb.rgb_data[i].r;
        data_b = rgb.rgb_data[i].b;
        for (int j = 0; j < 8; j++) {
            rgb.rgb_buffer[24 * i + 7 - j + ZERO_NUM] = code[data_g & 0x01];
            rgb.rgb_buffer[24 * i + 15 - j + ZERO_NUM] = code[data_r & 0x01];
            rgb.rgb_buffer[24 * i + 23 - j + ZERO_NUM] = code[data_b & 0x01];
            data_g >>= 1; data_r >>= 1; data_b >>= 1;
        }
    }
    HAL_SPI_Transmit_DMA(&hspi1, rgb.rgb_buffer, 24 * reflash_num  + 2 * ZERO_NUM);
}

void rgb_change(uint8_t i, uint8_t status)
{
    rgb.rgb_status[i] = status;
}

void rgb_set_bright(uint8_t i, uint8_t bright)
{
    rgb_set_color(i, (rgb_color_t){bright,bright,bright});
}

void rgb_output_data(void)
{
    for (int i = 0; i < RGB_NUM; i++) {
        switch (rgb.rgb_status[i]) {
            case 0: rgb_set_color(i, (rgb_color_t){0,0,0});break;                           //无色
            case 1: rgb_set_color(i, (rgb_color_t){brightness,0,0});break;                  //红色
            case 2: rgb_set_color(i, (rgb_color_t){0,brightness,0});break;                  //绿色
            case 3: rgb_set_color(i, (rgb_color_t){0,0,brightness});break;                  //蓝色
            case 4: rgb_set_color(i, (rgb_color_t){brightness,brightness,0});break;         //黄色
            case 5: rgb_set_color(i, (rgb_color_t){brightness,0,brightness});break;         //紫色
            case 6: rgb_set_color(i, (rgb_color_t){0,brightness,brightness});break;         //靛色
            case 7: rgb_set_color(i, (rgb_color_t){brightness,brightness,brightness});break;//白色
            case 8: break;
        }
    }
    rgb_reflash(RGB_NUM);
}
