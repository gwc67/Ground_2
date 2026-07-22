#ifndef __ANO_DEVICE_GROUND_H
#define __ANO_DEVICE_GROUND_H

#include "main.h"

/* ============== 发送侧数据结构（保留） ============== */

struct Animal_Report_Data_t {
    uint8_t row_uc;//Y轴位置
    uint8_t col_uc;//X轴位置
    uint8_t count_per_type_auc[6];//数量信息
}__attribute__ ((__packed__));

/* ============== 发送侧接口 ============== */

void vGround_DT_Data_Receive_Anl_Ano(uint8_t *pucdata, uint8_t uclen);
void vGround_Add_Send_Data_Ano(uint8_t ucFrame_num, uint8_t *pcnt, uint8_t *pucTxBuffer);
void vGround_TxBuffer_Ano(uint8_t *pucData, uint8_t ucLength);
void vGround_Data_Exchange_Task_Ano(void);
void Ground_Set_ALT_FU_v(int32_t ALT_FU_l );
void Ground_Set_Voltage_v(uint16_t Voltage_100_us );
void Ground_Set_Current_v(uint16_t Current_100_us );
void ground_send_animal_report_v(const struct Animal_Report_Data_t *report_st);

/* =====================================================================
 * 接收侧（地面站作为接收端，解析飞控发来的 9 个数据帧）
 *
 * 载荷字节序与 FC 端 vGround_Add_Send_Data_Ano 发送顺序一致。
 * setter 为纯函数（仅写静态存储），可独立测试。
 * copy 为线程安全快照。
 * ===================================================================== */

/* 帧号（与 FC 端定义对应） */


/* FC 相位枚举（与 FC fly_task_phase_e 严格对应）
 * WAITING_PATROL / WAITING_RETURN 是 FC 的握手机制：
 * FC 进入这两个状态 = 告诉 GCS "我准备好接收航点了"
 */
#define FC_PHASE_IDLE             0
#define FC_PHASE_TAKEOFF_DELAY    1
#define FC_PHASE_ALT_HOLD         2
#define FC_PHASE_WAITING_PATROL   3   /* 新增：FC 等待巡逻航点下发 */
#define FC_PHASE_PATROL           4
#define FC_PHASE_WAITING_RETURN   5   /* 新增：FC 等待返航航点下发 */
#define FC_PHASE_RETURN           6
#define FC_PHASE_DESCEND          7
#define FC_PHASE_LAND             8
#define FC_PHASE_DONE             9

/* 接收数据结构（packed，与 FC 发送载荷字节对齐） */
//雷达位置信息
struct gs_radar_pos_t {
    int16_t x_x100_s;
    int16_t y_x100_s;
    int16_t z_x100_s;
    int16_t freq_x10_s;
} __attribute__((__packed__));
//雷达速度信息
struct gs_radar_speed_t {
    int16_t x_x100_s;
    int16_t y_x100_s;
    int16_t z_x100_s;
    int16_t freq_x10_s;
} __attribute__((__packed__));

//雷达偏航角
struct gs_radar_yaw_t {
    int16_t yaw_s;
    int16_t freq_x10_s;
} __attribute__((__packed__));
//pwm值
struct gs_pwm_t {
    uint16_t pwm_m1;
    uint16_t pwm_m2;
    uint16_t pwm_m3;
    uint16_t pwm_m4;
} __attribute__((__packed__));
//电压、电流、高度、阶段
struct gs_batt_curr_height_process_t {
    uint16_t voltage_100;
    uint16_t current_100;
    uint32_t alt_fu_ul;
    uint8_t  process_uc;
} __attribute__((__packed__));
//地面站下发的控制信息
struct gs_cmd_vel_t {
    int16_t rol;
    int16_t pit;
    int16_t thr;
    int16_t yaw_dps;
    int16_t vel_x;
    int16_t vel_y;
    int16_t vel_z;
    int16_t freq_x10_s;
} __attribute__((__packed__));
//地面巡查航线速度配置参数
struct gs_vel_fu_t {
    int16_t vel_x_s;
    int16_t vel_y_s;
    int16_t vel_z_s;
    int16_t freq_x10_s;
} __attribute__((__packed__));





/* copy — 线程安全快照（供其他模块读取） */
void radar_pos_copy                (struct gs_radar_pos_t *out);
void radar_speed_copy              (struct gs_radar_speed_t *out);
void radar_yaw_copy                (struct gs_radar_yaw_t *out);
void pwm_copy                      (struct gs_pwm_t *out);
void batt_curr_height_process_copy (struct gs_batt_curr_height_process_t *out);
void cmd_vel_copy                  (struct gs_cmd_vel_t *out);
void vel_fu_copy                   (struct gs_vel_fu_t *out);

/* ============== 发送侧 — 航点上传接口 ============== */

#include "Common/types.h"

/**
 * @brief 清空飞控航点 FIFO (0xE0/0x01, CMD 帧, 等 ACK)
 */

/**
 * @brief 完整任务巡逻航点上传
 * @param patrol_pts 巡逻航点
 * @param patrol_cnt 巡逻航点数量 (≤63)
 *
 * 流程：缓存航点 → 发 0xE0/0x01 clear (等 ACK) → ACK 到达后自动发 0x16 航点帧
 */
void ground_send_patrol_waypoints_v(const struct Point_t *patrol_pts, uint8_t patrol_cnt);

/**
 * @brief 返航航点上传
 *
 * 流程同 patrol：缓存 → clear (0xE0/0x01) → ACK 后发 0x16
 */
void ground_send_return_waypoints_v(const struct Point_t *return_pts, uint8_t return_cnt);

#endif
