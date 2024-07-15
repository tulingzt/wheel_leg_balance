#ifndef __PROT_DR16_H
#define __PROT_DR16_H

#include "stm32h7xx.h"

#define DR16_DATA_LEN 18

#define RC_LEFT_LU  ( 1<<0 ) //居左上
#define RC_LEFT_RU  ( 1<<1 ) //居右上
#define RC_LEFT_RD  ( 1<<2 ) //居右下
#define RC_LEFT_LD  ( 1<<3 ) //居左下
#define RC_RIGHT_LU ( 1<<4 ) //居左上
#define RC_RIGHT_RU ( 1<<5 ) //居右上
#define RC_RIGHT_RD ( 1<<6 ) //居右下
#define RC_RIGHT_LD ( 1<<7 ) //居左下

typedef __packed struct
{
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	uint8_t sw1;
	uint8_t sw2;
	__packed struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;
	} mouse;
	__packed union
	{
		uint16_t key_code;
		__packed struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;//16个键位
		} bit;
	} kb;
    uint8_t init_status;//遥控器上电拨杆状态机
} dr16_t;

typedef enum
{
    KB_Q = 0,
    KB_E = 1,
    KB_R = 2,
    KB_F = 3,
    KB_G = 4,
    KB_Z = 5,
    KB_X = 6,
    KB_C = 7,
    KB_V = 8,
    KB_B = 9,
    KB_CTRL = 10,
    KB_SHIFT = 11,
    KB_NULL = 12  //用户不想使用的按键集
} key_index_e;

typedef enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
} rc_sw_mode_e;

typedef enum
{
    KEY_RUN = 1,
    KEY_END = 0
} rc_key_status_e;

extern dr16_t rc;
extern int kb_status[13];

uint8_t dr16_get_data(dr16_t *rc, uint8_t *data);
uint8_t key_scan(key_index_e key_index);
void key_status_clear(key_index_e key_index);
uint8_t key_scan_clear(key_index_e key_index);
void rc_fsm_init(uint8_t trig_flag);
uint8_t rc_fsm_check(uint8_t target_status);

#endif
