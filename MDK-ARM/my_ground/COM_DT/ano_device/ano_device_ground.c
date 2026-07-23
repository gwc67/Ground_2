/* ============================================================================
 * ano_device_ground.c — 地面站 USART1 ANO 协议设备层
 *
 * ★ 这是你日常改协议最常碰的文件。改"收什么帧/发什么帧"都在这里。
 * 它被 ano_ture.c 的状态机调用：收满一帧 → Ano_Receive_Anl(本文件)；
 * 要发某帧 → vFrame_Send_Ano 调 Ano_Add_Send_Data(本文件)填载荷。
 *
 * USART1 连接飞控 USART6，走 ANO 协议。
 * 已移除对飞控内部模块（Route_Planning / Freq_Detector / OF 数据）的依赖，
 * 仅保留地面站自身的功能。
 *
 * 文件分三块（对应 unique 表的三个函数指针）：
 *   收：vGround_DT_Data_Receive_Anl_Ano  — 收满整帧后，校验并按 CMD 分发
 *   填：vGround_Add_Send_Data_Ano       — 发送时，按帧号填充载荷字节
 *   发：vGround_TxBuffer_Ano            — 物理层发出(调 uart_transmit)
 * ========================================================================== */
#include "ano_device_ground.h"
#include "ano.h"
#include "driver_registry.h"
#include "Mission/mission_planner.h"
#include "string.h"
#include "stdbool.h"
#include "update.h"
#include "Report/report.h"
#include "point_3d.h"

/* 飞控一次最大接受航点数（UART RX 缓冲区限制，10 个 × 4 字节 + 帧开销 < 64 字节） */
#define MAX_WAYPOINTS_PER_FRAME  10
/* 航点总数上限 */
#define MAX_WAYPOINTS_TOTAL      63

/* ============== 帧号定义 ============== */

#define RADAR_POS                   0x01
#define RADAR_SPEED                 0x02
#define RADAR_YAW                   0x04
#define PWM                         0x10
#define BATT_CURR_HEIGHT_PROCESS    0x11
#define CMD_VEL                     0x12
#define VEL_FU                      0x13
#define REPORT                      0x14

#define POINT_PATROL                0x16
#define POINT_RETURN                0x17

#define GROUND_REQUEST_PATROL       0x18
#define GROUND_REQUEST_RETURN       0x19

#define CLEAR_POINT                 0x20     
#define GROUND_SPECIAL_DELIVRY      0x21     //准备查找的指定货物


/* ============== 数据结构 ============== */

static struct Animal_Report_Data_t s_animal_report_st;

/* 航点发送缓存（0x16 数据帧，分批发送） */
// static struct Point_t s_waypoint_tx_buf[MAX_WAYPOINTS_TOTAL];
// static uint8_t s_waypoint_total;         /* 总航点数 */
// static uint8_t s_waypoint_batch_off;     /* 当前批次起始索引 */
// static uint8_t s_waypoint_batches_left;  /* 剩余批次数 */


/* ============== 接收侧 — 飞控发来的 9 个数据帧存储 ============== */

static struct gs_radar_pos_t                s_radar_pos;
static struct gs_radar_speed_t              s_radar_speed;
static struct gs_radar_yaw_t                s_radar_yaw;
static struct gs_pwm_t                      s_pwm;
static struct gs_batt_curr_height_process_t s_batt_curr_height_process;
static struct gs_cmd_vel_t                  s_cmd_vel;
static struct gs_vel_fu_t                   s_vel_fu;


void radar_pos_copy(struct gs_radar_pos_t *out)
{
    *out = s_radar_pos;
}

void radar_speed_copy(struct gs_radar_speed_t *out)
{
    *out = s_radar_speed;
}


void radar_yaw_copy(struct gs_radar_yaw_t *out)
{
    *out = s_radar_yaw;
}

void pwm_copy(struct gs_pwm_t *out)
{
    *out = s_pwm;
}

void batt_curr_height_process_copy(struct gs_batt_curr_height_process_t *out)
{
    *out = s_batt_curr_height_process;
}

void cmd_vel_copy(struct gs_cmd_vel_t *out)
{
    *out = s_cmd_vel;
}

void vel_fu_copy(struct gs_vel_fu_t *out)
{
    *out = s_vel_fu;
}



/* ============== 帧处理函数（封装每类 CMD 的逻辑） ============== */
/* payload 指向 pucdata + 4（载荷首字节），check_sum1/sum2 用于 ACK 回执 */

/* 0xE0 — 命令帧：按子命令分发，回 CK 校验回执 */
static void handle_cmd_frame(const uint8_t *payload, u8 check_sum1, u8 check_sum2)
{
    switch (*payload)
    {
    case 0x01: break;
    case 0x02: break;
    case 0x10: break;  /* 禁飞区数据 — 地面站暂不处理 */
    case 0x11: break;
    default:   break;
    }
    ano_ck_back_v(pstAnobase_Ground, 0xff, 0xE0, check_sum1, check_sum2);
}

/* 0xE1 — 读参数帧：回 par_back + CK */
static void handle_par_read_frame(const uint8_t *payload, u8 check_sum1, u8 check_sum2)
{
    uint16_t par_id = payload[0] + (uint16_t)payload[1] * 256;
    ano_par_back_v(pstAnobase_Ground, 0xff, par_id, 0);
    ano_ck_back_v(pstAnobase_Ground, 0xff, 0xE1, check_sum1, check_sum2);
}

/* 0xE2 — 参数写入帧：回 CK */
static void handle_par_write_frame(u8 check_sum1, u8 check_sum2)
{
    ano_ck_back_v(pstAnobase_Ground, 0xff, 0xE2, check_sum1, check_sum2);
}

/* 0x00 — 确认帧：比对 ck_send，匹配则清 wait_ck */
static void handle_ack_frame(const uint8_t *payload)
{
    if (ano_ck_id_get(pstAnobase_Ground) == payload[0] &&
        ano_ck_sc_get(pstAnobase_Ground) == payload[1] &&
        ano_ck_ac_get(pstAnobase_Ground) == payload[2])
    {
        vano_wait_ck_clear(pstAnobase_Ground);
    }
}

/* ============== 初始化 — 设置帧发送频率 ============== */

void vGround_init_Ano(void)
{
    vano_sendID_set(pstAnobase_Ground, 0x00, 0);
    vano_sendID_set(pstAnobase_Ground, POINT_PATROL, 0);       /* 外部触发发送 */
    vano_sendID_set(pstAnobase_Ground, POINT_RETURN, 0);       /* 外部触发发送 */

}
DRIVER_INIT(vGround_init_Ano);

/* ============== 接收帧解析 ==============
 * ★ 状态机收满一整帧后调本函数(经 unique.Ano_Receive_Anl)。
 * pucdata 指向整帧：[0]=0xAA [1]=0xFF [2]=CMD [3]=LEN [4..]=载荷 [后2]=校验
 * uclen = 整帧总字节数。
 *
 * 三步：
 *   1) 校验 LEN 字段一致(pucdata[3] 应等于 uclen-6)
 *   2) 算两级累加校验和，比对帧尾 SUM1/SUM2，不符就丢弃(防错帧)
 *   3) 按 CMD(pucdata[2]) 分发：
 *      - 数据帧(0x01..0x17)：飞控→地面站，直接 memcpy 存进静态变量，
 *        不回 ACK(fire-and-forget，量大丢一帧无所谓)
 *      - 命令帧(0xE0/0xE1/0xE2)：要回 ACK 校验回执(要确认对方执行了)
 *      - 确认帧(0x00)：对方回的 ACK，比对成功就 vano_wait_ck_clear */
void vGround_DT_Data_Receive_Anl_Ano(uint8_t *pucdata, uint8_t uclen)
{
    u8 check_sum1 = 0, check_sum2 = 0;

    /* === 接缝 1: 校验和验证 === */
    if (*(pucdata + 3) != (uclen - 6))
        return;
    for (u8 i = 0; i < uclen - 2; i++)
    {
        check_sum1 += *(pucdata + i);
        check_sum2 += check_sum1;
    }
    if ((check_sum1 != *(pucdata + uclen - 2)) || (check_sum2 != *(pucdata + uclen - 1)))
        return;
    if (*(pucdata) != 0xAA || *(pucdata + 1) != 0xFF)
        return;

    uint8_t cmd = *(pucdata + 2);
    const uint8_t *payload = pucdata + 4;

    switch (cmd)
    {
    /* -------- 数据帧（飞控 → 地面站，被动接收不回 ACK） -------- */
    case RADAR_POS: {
        memcpy(&s_radar_pos, payload, sizeof(s_radar_pos));
    }
    break;
    case RADAR_SPEED: {
        memcpy(&s_radar_speed, payload, sizeof(s_radar_speed));
    }
    break;
    case RADAR_YAW: {
        memcpy(&s_radar_yaw, payload, sizeof(s_radar_yaw));
    }
    break;
    case PWM: {
        memcpy(&s_pwm, payload, sizeof(s_pwm));
    }
    break;
    case BATT_CURR_HEIGHT_PROCESS: {
        memcpy(&s_batt_curr_height_process, payload, sizeof(s_batt_curr_height_process));
    }
    break;

    case CMD_VEL: {
        memcpy(&s_cmd_vel, payload, sizeof(s_cmd_vel));
    }
    break;
    case VEL_FU: {
        memcpy(&s_vel_fu, payload, sizeof(s_vel_fu));
    }
    break;
    case REPORT: {
        struct delivery_t temp_st = {0};
        memcpy(&temp_st, payload, sizeof(temp_st));
        delivery_add_b(&temp_st);
    }
    break;
    case GROUND_REQUEST_PATROL: {
        update_flag_set_v(UPDATE_FLAG_REQUEST_PATROL_em);
    }
    break;
    case GROUND_REQUEST_RETURN: {
        update_flag_set_v(UPDATE_FLAG_REQUEST_RETURN_em);
    }
    break;
    case GROUND_SPECIAL_DELIVRY:
    {
        struct delivery_t temp_st = {0};
        memcpy(&temp_st, payload, sizeof(temp_st));
        delivery_set_special(&temp_st);
        update_flag_set_v(UPDATE_FLAG_DELVIERY_SPECIAL_em);         //更新新货物
    }
    break;
    /* -------- 命令帧（地面站需回 ACK 响应） -------- */
    case 0xE0:
        handle_cmd_frame(payload, check_sum1, check_sum2);
        break;
    case 0xE1:
        handle_par_read_frame(payload, check_sum1, check_sum2);
        break;
    case 0xE2:
        handle_par_write_frame(check_sum1, check_sum2);
        break;

    /* -------- 确认帧 -------- */
    case 0x00:
        handle_ack_frame(payload);
        break;
    default:
        break;
    }
}

/* ============== 发送数据填充 ==============
 * ★ vFrame_Send_Ano(ano_ture.c) 拼帧时调本函数(经 unique.Ano_Add_Send_Data)。
 * 职责：按帧号(ucFrame_num)把要发的载荷字节追加到 pucTxBuffer，
 * 通过 *pcnt 追加写入位置(函数会累加 *pcnt)。
 * 帧头/LEN/校验由 ano_ture.c 统一处理，这里只管"载荷内容"。
 *
 * 要新增一种"发送数据帧"？在下面 switch 加一个 case 就行。 */
void vGround_Add_Send_Data_Ano(uint8_t ucFrame_num, uint8_t *pcnt, uint8_t *pucTxBuffer)
{
    switch (ucFrame_num)
    {

    case 0x00: {
        pucTxBuffer[(*pcnt)++] = ano_ck_id_get(pstAnobase_Ground);
        pucTxBuffer[(*pcnt)++] = ano_ck_sc_get(pstAnobase_Ground);
        pucTxBuffer[(*pcnt)++] = ano_ck_ac_get(pstAnobase_Ground);
    }
    break;


    case REPORT:
    {
        memcpy(pucTxBuffer + *(pcnt), &s_animal_report_st, sizeof(s_animal_report_st));
        *pcnt += sizeof(s_animal_report_st);
    }
    break;
    case POINT_PATROL:
    {
        struct Point_3D_t snap = {0};
        point_3d_take_t(g_partrol_point_3d_pst,&snap);
        memcpy(pucTxBuffer + *(pcnt), &snap,sizeof(snap));
        *pcnt += sizeof(snap);
    }
    break;
    case POINT_RETURN:
    {
        struct Point_3D_t snap = {0};
        point_3d_take_t(g_return_point_3d_pst,&snap);
        memcpy(pucTxBuffer + *(pcnt), &snap,sizeof(snap));
        *pcnt += sizeof(snap);
    }
    break;
    case CLEAR_POINT:       break;
    default:
        break;
    }
}

void vGround_TxBuffer_Ano(uint8_t *pucData, uint8_t ucLength)
{
    uart_transmit(pstbase_ground_uart, pucData, ucLength);
}


void vGround_Data_Exchange_Task_Ano(void)
{
    vano_ck_back_check(pstAnobase_Ground);
    vano_check_to_send(pstAnobase_Ground, 0x00);
    vano_check_to_send(pstAnobase_Ground, 0xe0);
    vano_check_to_send(pstAnobase_Ground, REPORT);
    vano_check_to_send(pstAnobase_Ground, POINT_PATROL);
    vano_check_to_send(pstAnobase_Ground, POINT_RETURN);
}



void ground_send_animal_report_v(const struct Animal_Report_Data_t *report_st)
{
    s_animal_report_st = *report_st;
    vano_WTS_set(pstAnobase_Ground, REPORT, 1);
}


//添加地面站的巡逻航点
// void ground_send_patrol_waypoints_v(const struct Point_t *patrol_pts, uint8_t patrol_cnt)
// {
//     waypoint_send_prepare(patrol_pts, patrol_cnt);
// }

// //添加地面站算出来的回家航点
// void ground_send_return_waypoints_v(const struct Point_t *return_pts, uint8_t return_cnt)
// {
//     waypoint_send_prepare(return_pts, return_cnt);
// }
