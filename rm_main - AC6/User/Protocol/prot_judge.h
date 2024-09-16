#ifndef __BSP_JUDGE_H
#define __BSP_JUDGE_H

#include "usart.h"
#include "stdint.h"

//帧头
typedef __PACKED_STRUCT
{
    uint8_t SOF;            //数据帧起始字节，固定值为 0xA5
    uint16_t data_length;   //数据帧中 data 的长度
    uint8_t seq;            //包序号
    uint8_t CRC8;           //帧头 CRC8 校验
} frame_header_t;

//命令码
typedef enum
{
    ID_game_status                  = 0x0001, //比赛状态数据，固定以1Hz频率发送
    ID_game_result                  = 0x0002, //比赛结果数据，比赛结束触发发送
    ID_game_robot_HP                = 0x0003, //机器人血量数据，固定以3Hz频率发送
    ID_event_data                   = 0x0101, //场地事件数据，固定以1Hz频率发送
    ID_ext_supply_projectile_action = 0x0102, //补给站动作标识数据，补给站弹丸释放时触发发送
    ID_referee_warning              = 0x0104, //裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送
    ID_dart_info                    = 0x0105, //飞镖发射相关数据，固定以1Hz频率发送

    ID_robot_status                 = 0x0201, //机器人性能体系数据，固定以10Hz频率发送
    ID_power_heat_data              = 0x0202, //实时底盘功率和枪口热量数据，固定以50Hz频率发送
    ID_robot_pos                    = 0x0203, //机器人位置数据，固定以1Hz频率发送
    ID_buff                         = 0x0204, //机器人增益数据，固定以3Hz频率发送
    ID_air_support_data             = 0x0205, //空中支援时间数据，固定以1Hz频率发送
    ID_hurt_data                    = 0x0206, //伤害状态数据，伤害发生后发送
    ID_shoot_data                   = 0x0207, //实时射击数据，弹丸发射后发送
    ID_projectile_allowance         = 0x0208, //允许发弹量，固定以10Hz频率发送
    ID_rfid_status                  = 0x0209, //机器人RFID模块状态，固定以3Hz频率发送
    ID_dart_client_cmd              = 0x020A, //飞镖选手端指令数据，固定以3Hz频率发送
    ID_ground_robot_position        = 0x020B, //地面机器人位置数据，固定以1Hz频率发送
    ID_radar_mark_data              = 0x020C, //雷达标记进度数据，固定以1Hz频率发送
    ID_sentry_info                  = 0x020D, //哨兵自主决策信息同步，固定以1Hz频率发送
    ID_radar_info                   = 0x020E, //雷达自主决策信息同步，固定以1Hz频率发送

    ID_robot_interaction_data       = 0x0301, //机器人交互数据，发送方触发发送，频率上限为10Hz
    ID_custom_robot_data            = 0x0302, //自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz        图传链路
    ID_map_command                  = 0x0303, //选手端小地图交互数据，选手端触发发送
    ID_remote_control               = 0x0304, //键鼠遥控数据，固定30Hz频率发送                                      图传链路
    ID_map_robot_data               = 0x0305, //选手端小地图接收雷达数据，频率上限为10Hz
    ID_custom_client_data           = 0x0306, //自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz
    ID_map_data                     = 0x0307, //选手端小地图接收哨兵数据，频率上限为1Hz
    ID_custom_info                  = 0x0308  //选手端小地图接收机器人数据，频率上限为3Hz
} cmd_id_e;

//数据段长度
typedef enum
{
    LEN_game_status                 = 11,
    LEN_game_result                 = 1,
    LEN_game_robot_HP               = 32,

    LEN_event_data                  = 4,
    LEN_ext_supply_projectile_action= 4,
    LEN_referee_warning             = 3,
    LEN_dart_info                   = 3,

    LEN_robot_status                = 13,
    LEN_power_heat_data             = 16,
    LEN_robot_pos                   = 12,//手册有矛盾 有16和12
    LEN_buff                        = 6,
    LEN_air_support_data            = 2,
    LEN_hurt_data                   = 1,
    LEN_shoot_data                  = 7,
    LEN_projectile_allowance        = 6,
    LEN_rfid_status                 = 4,
    LEN_dart_client_cmd             = 6,
    LEN_ground_robot_position       = 40,
    LEN_radar_mark_data             = 6,
    LEN_sentry_info                 = 4,
    LEN_radar_info                  = 1,

    LEN_robot_interaction_data      = 128,
    LEN_custom_robot_data           = 30,
    LEN_map_command                 = 12,//手册有矛盾 有15和12
    LEN_remote_control              = 12,
    LEN_map_robot_data              = 10,
    LEN_custom_client_data          = 8,
    LEN_map_data                    = 105,//手册有矛盾 有103和105
    LEN_custom_info                 = 34
} cmd_len_e;

//各数据结构体定义
typedef __PACKED_STRUCT
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef __PACKED_STRUCT
{
    uint8_t winner;
} game_result_t;

typedef __PACKED_STRUCT
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_6_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_6_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

typedef __PACKED_STRUCT
{
    uint32_t event_data;
} event_data_t;

typedef __PACKED_STRUCT
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __PACKED_STRUCT
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef __PACKED_STRUCT
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

typedef __PACKED_STRUCT
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef __PACKED_STRUCT
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef __PACKED_STRUCT
{
    float x;
    float y;
    float angle;
} robot_pos_t;

typedef __PACKED_STRUCT
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} buff_t;

typedef __PACKED_STRUCT
{
    uint8_t airforce_status;
    uint8_t time_remain;
} air_support_data_t;

typedef __PACKED_STRUCT
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef __PACKED_STRUCT
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

typedef __PACKED_STRUCT
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} projectile_allowance_t;

typedef __PACKED_STRUCT
{
    uint32_t rfid_status;
} rfid_status_t;

typedef __PACKED_STRUCT
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef __PACKED_STRUCT
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float enginerr_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} ground_robot_position_t;

typedef __PACKED_STRUCT
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} radar_mark_data_t;

typedef __PACKED_STRUCT
{
    uint32_t sentry_info;
} sentry_info_t;

typedef __PACKED_STRUCT
{
    uint8_t radar_info;
} radar_info_t;

typedef __PACKED_STRUCT
{
    uint8_t data[30];
} custom_robot_data_t;

typedef __PACKED_STRUCT
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} map_command_t;

typedef __PACKED_STRUCT
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} remote_control_t;

typedef __PACKED_STRUCT
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} map_robot_data_t;

typedef __PACKED_STRUCT
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} custom_client_data_t;

typedef __PACKED_STRUCT
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

typedef __PACKED_STRUCT
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;

//机器人交互数据
typedef __PACKED_STRUCT
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];
} robot_interaction_data_t;

typedef enum
{
    ID_robot_interaction    = 0x0200,
    ID_delete_figure        = 0x0100,
    ID_draw_one_figure      = 0x0101,
    ID_draw_two_fiugre      = 0x0102,
    ID_draw_five_figure     = 0x0103,
    ID_draw_seven_fiugre    = 0x0104,
    ID_draw_character_figure= 0x0110,
    ID_sentry_cmd           = 0x0120,
    ID_radar_cmd            = 0x0121
} robot_interaction_id_e;

typedef enum
{
    LEN_robot_interaction    = 113,
    LEN_delete_figure        = 2,
    LEN_draw_one_figure      = 15,
    LEN_draw_two_fiugre      = 30,
    LEN_draw_five_figure     = 75,
    LEN_draw_seven_fiugre    = 105,
    LEN_draw_character_figure= 45,
    LEN_sentry_cmd           = 4,
    LEN_radar_cmd            = 1
} robot_interaction_len_e;

//0x0200-0x02FF
//0x0100 选手端删除图层
typedef __PACKED_STRUCT
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

//0x0101 选手端绘制一个图形
typedef __PACKED_STRUCT
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;

//0x0102 选手端绘制两个图形
typedef __PACKED_STRUCT
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

//0x0103 选手端绘制五个图形
typedef __PACKED_STRUCT
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

//0x0104 选手端绘制七个图形
typedef __PACKED_STRUCT
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

//0x0110 选手端绘制字符图形
typedef __PACKED_STRUCT
{
    interaction_figure_t interaction_figure;
    uint8_t data[30];
} ext_client_custom_character_t;

//0x0120 哨兵自主决策指令
typedef __PACKED_STRUCT
{
    uint32_t sentry_cmd;
} sentry_cmd_t;

//0x0121 雷达自主决策指令
typedef __PACKED_STRUCT
{
    uint8_t radar_cmd;
} radar_cmd_t;

extern frame_header_t frame_header;

extern game_status_t game_status;
extern robot_status_t robot_status;
extern shoot_data_t shoot_data;
extern power_heat_data_t power_heat_data;

void judge_init(UART_HandleTypeDef *huart);
uint8_t judge_get_data(uint8_t *data);
void judge_send_data(uint8_t* message, int length);
uint8_t judge_check_offline(void);

#endif
