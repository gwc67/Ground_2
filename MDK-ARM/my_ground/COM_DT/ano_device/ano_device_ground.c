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
/* 飞控一次最大接受航点数（UART RX 缓冲区限制，10 个 × 4 字节 + 帧开销 < 64 字节） */
#define MAX_WAYPOINTS_PER_FRAME  10
/* 航点总数上限 */
#define MAX_WAYPOINTS_TOTAL      63

/* ============== 帧号定义 ============== */


/* ============== 数据结构 ============== */

static struct Animal_Report_Data_t s_animal_report_st;

/* 航点发送缓存（0x16 数据帧，分批发送） */
static struct Point_t s_waypoint_tx_buf[MAX_WAYPOINTS_TOTAL];
static uint8_t s_waypoint_total;         /* 总航点数 */
static uint8_t s_waypoint_batch_off;     /* 当前批次起始索引 */
static uint8_t s_waypoint_batches_left;  /* 剩余批次数 */


/* ============== 接收侧 — 飞控发来的 9 个数据帧存储 ============== */

static struct gs_radar_pos_t                s_radar_pos;
static struct gs_radar_speed_t              s_radar_speed;
static struct gs_radar_yaw_t                s_radar_yaw;
static struct gs_pwm_t                      s_pwm;
static struct gs_batt_curr_height_process_t s_batt_curr_height_process;
static struct gs_cmd_vel_t                  s_cmd_vel;
static struct gs_vel_fu_t                   s_vel_fu;



/* ============== 接收侧 — setter 纯函数（载荷字节 → 存储） ============== */
/* payload 指向 pucdata + 4（即紧跟 CMD + LEN 之后的载荷首字节） */

void gs_radar_pos_set(const uint8_t *payload)
{
    memcpy(&s_radar_pos, payload, sizeof(s_radar_pos));
}

void gs_radar_speed_set(const uint8_t *payload)
{
    memcpy(&s_radar_speed, payload, sizeof(s_radar_speed));
}


void gs_radar_yaw_set(const uint8_t *payload)
{
    memcpy(&s_radar_yaw, payload, sizeof(s_radar_yaw));
}

void gs_pwm_set(const uint8_t *payload)
{
    memcpy(&s_pwm, payload, sizeof(s_pwm));
}

void gs_batt_curr_height_process_set(const uint8_t *payload)
{
    memcpy(&s_batt_curr_height_process, payload, sizeof(s_batt_curr_height_process));
}

void gs_cmd_vel_set(const uint8_t *payload)
{
    memcpy(&s_cmd_vel, payload, sizeof(s_cmd_vel));
}

void gs_vel_fu_set(const uint8_t *payload)
{
    memcpy(&s_vel_fu, payload, sizeof(s_vel_fu));
}

void gs_fc_clear_flag_success(const uint8_t *payload)
{
    /* 飞控回 0x17/0x01 确认 clear 成功。
     * 批次链式发送由 Add_Send_Data 内部状态机驱动，无需在此触发 WTS。 */
    (void)payload;
}
/* ============== 接收侧 — copy 线程安全快照 ============== */

void gs_radar_pos_copy(struct gs_radar_pos_t *out)
{
    *out = s_radar_pos;
}

void gs_radar_speed_copy(struct gs_radar_speed_t *out)
{
    *out = s_radar_speed;
}


void gs_radar_yaw_copy(struct gs_radar_yaw_t *out)
{
    *out = s_radar_yaw;
}

void gs_pwm_copy(struct gs_pwm_t *out)
{
    *out = s_pwm;
}

void gs_batt_curr_height_process_copy(struct gs_batt_curr_height_process_t *out)
{
    *out = s_batt_curr_height_process;
}

void gs_cmd_vel_copy(struct gs_cmd_vel_t *out)
{
    *out = s_cmd_vel;
}

void gs_vel_fu_copy(struct gs_vel_fu_t *out)
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
    vano_sendID_set(pstAnobase_Ground, GS_FRAME_REPORT, 0);  /* 外部触发发送 */
    vano_sendID_set(pstAnobase_Ground, GS_FRAME_WAYPOINT, 0);       /* 外部触发发送 */
    vano_sendID_set(pstAnobase_Ground, GS_FRAME_WAYPOINT_CLEAR, 0); /* 外部触发发送 */
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
    case GS_FRAME_RADAR_POS:                gs_radar_pos_set(payload);                break;
    case GS_FRAME_RADAR_SPEED:              gs_radar_speed_set(payload);              break;
    case GS_FRAME_RADAR_YAW:                gs_radar_yaw_set(payload);                break;
    case GS_FRAME_PWM:                      gs_pwm_set(payload);                      break;
    case GS_FRAME_BATT_CURR_HEIGHT_PROCESS: gs_batt_curr_height_process_set(payload); break;
    case GS_FRAME_CMD_VEL:                  gs_cmd_vel_set(payload);                  break;
    case GS_FRAME_VEL_FU:                   gs_vel_fu_set(payload);                   break;
    case GS_FRAME_WAYPOINT_CLEAR:           gs_fc_clear_flag_success(payload);        break;
    case GS_FRAME_REPORT:
    {
        struct delivery_t temp_st;
        memcpy(&temp_st,payload,sizeof(temp_st));
        delivery_add_b(&temp_st);
    }
    break;
    /* -------- 命令帧（地面站需回 ACK 响应） -------- */
    case 0xE0: handle_cmd_frame(payload, check_sum1, check_sum2);       break;
    case 0xE1: handle_par_read_frame(payload, check_sum1, check_sum2);  break;
    case 0xE2: handle_par_write_frame(check_sum1, check_sum2);          break;

    /* -------- 确认帧 -------- */
    case 0x00: handle_ack_frame(payload);                               break;
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


    case GS_FRAME_REPORT:
    {
        memcpy(pucTxBuffer + *(pcnt), &s_animal_report_st, sizeof(s_animal_report_st));
        *pcnt += sizeof(s_animal_report_st);
    }
    break;

    case GS_FRAME_WAYPOINT:
    {
        /* 载荷格式：{count(1), Point_t[count] (每个 4 字节: x16+y16)} */
        uint8_t start = s_waypoint_batch_off;
        uint8_t remain = (s_waypoint_total > start) ? (s_waypoint_total - start) : 0;
        uint8_t count = (remain > MAX_WAYPOINTS_PER_FRAME) ? MAX_WAYPOINTS_PER_FRAME : remain;
        pucTxBuffer[(*pcnt)++] = count;
        for (uint8_t i = 0; i < count; i++)
        {
            pucTxBuffer[(*pcnt)++] = (uint8_t)(s_waypoint_tx_buf[start + i].x & 0xFF);
            pucTxBuffer[(*pcnt)++] = (uint8_t)((s_waypoint_tx_buf[start + i].x >> 8) & 0xFF);
            pucTxBuffer[(*pcnt)++] = (uint8_t)(s_waypoint_tx_buf[start + i].y & 0xFF);
            pucTxBuffer[(*pcnt)++] = (uint8_t)((s_waypoint_tx_buf[start + i].y >> 8) & 0xFF);
        }
        /* 链式触发下一批：check_to_send 在调用 Add_Send_Data 前已清 WTS，此处重设安全 */
        s_waypoint_batch_off += MAX_WAYPOINTS_PER_FRAME;
        s_waypoint_batches_left--;
        if (s_waypoint_batches_left > 0)
            vano_WTS_set(pstAnobase_Ground, GS_FRAME_WAYPOINT, 1);
    }
    break;

    case GS_FRAME_WAYPOINT_CLEAR:
    {   
        pucTxBuffer[(*pcnt)++] = 0x01;
    }
    break;

    default:
        break;
    }
}

/* ============== 物理层发送 ==============
 * ★ vFrame_Send_Ano 拼完整帧后调本函数(经 unique.Send_Buff)。
 * 直接转给 uart_transmit —— 这就是"两层 vtable 的咬合点 2"：
 *   ANO 层把整帧字节交给 UART 层发出。
 * uart_transmit 内部走 TX 队列非阻塞(DMA/IT)，立即返回不等发完。 */
void vGround_TxBuffer_Ano(uint8_t *pucData, uint8_t ucLength)
{
    uart_transmit(pstbase_ground_uart, pucData, ucLength);
}

/* ============== 周期发送调度 ==============
 * 任务循环(Ano_Scheduler.c → Ground_DT_UART)周期调本函数。
 * 三件事：
 *   1) vano_ck_back_check —— 检查命令帧 ACK 超时，该重传就重传
 *   2) vano_check_to_send(0x00) —— 发 ACK 回执帧(若 WTS 置位)
 *   3) vano_check_to_send(各数据帧) —— 按各自频率/标志决定发不发
 * 要新增周期发送的帧？在这里加一行 vano_check_to_send(帧号)。 */
void vGround_Data_Exchange_Task_Ano(void)
{
    vano_ck_back_check(pstAnobase_Ground);
    vano_check_to_send(pstAnobase_Ground, 0x00);
    vano_check_to_send(pstAnobase_Ground, 0xe0);
    vano_check_to_send(pstAnobase_Ground, GS_FRAME_REPORT);
    vano_check_to_send(pstAnobase_Ground, GS_FRAME_WAYPOINT);
    vano_check_to_send(pstAnobase_Ground, GS_FRAME_WAYPOINT_CLEAR);
}



void ground_send_animal_report_v(const struct Animal_Report_Data_t *report_st)
{
    s_animal_report_st = *report_st;
    vano_WTS_set(pstAnobase_Ground, GS_FRAME_REPORT, 1);
}

/* ============== 航点上传 — 数据帧 0x17 (clear) + 0x16 (航点) ============== */

void ground_send_waypoint_clear_v(void)
{
    vano_WTS_set(pstAnobase_Ground, GS_FRAME_WAYPOINT_CLEAR, 1);
}

/**
 * @brief 内部：缓存航点 → 发 clear (0x17) → 任务循环逐批发送 0x16
 */
static void waypoint_send_prepare(const struct Point_t *pts, uint8_t cnt)
{
    if (cnt > MAX_WAYPOINTS_TOTAL)
        cnt = MAX_WAYPOINTS_TOTAL;

    memcpy(s_waypoint_tx_buf, pts, cnt * sizeof(struct Point_t));
    s_waypoint_total = cnt;
    s_waypoint_batch_off = 0;
    s_waypoint_batches_left = (cnt + MAX_WAYPOINTS_PER_FRAME - 1) / MAX_WAYPOINTS_PER_FRAME;

    /* 发 clear (0x17) + 直接触发第一批 0x16 */
    ground_send_waypoint_clear_v();
    vano_WTS_set(pstAnobase_Ground, GS_FRAME_WAYPOINT, 1);
}

//添加地面站的巡逻航点
void ground_send_patrol_waypoints_v(const struct Point_t *patrol_pts, uint8_t patrol_cnt)
{
    waypoint_send_prepare(patrol_pts, patrol_cnt);
}

//添加地面站算出来的回家航点
void ground_send_return_waypoints_v(const struct Point_t *return_pts, uint8_t return_cnt)
{
    waypoint_send_prepare(return_pts, return_cnt);
}
