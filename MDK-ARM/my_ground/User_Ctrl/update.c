#include "update.h"
#include "stdio.h"
#include "stdbool.h"

struct update_flag_field {
    const char *name;
    enum update_state_e  val_em;
};

static struct update_flag_field s_fields[] = {
    [UPDATE_FLAG_REPORT_em] = {.name = "report",.val_em = UPDATE_STATE_NO_DATA_em},
    [UPDATE_FLAG_REQUEST_PATROL_em] = {.name = "request_patrol",.val_em = UPDATE_STATE_NO_DATA_em},
    [UPDATE_FLAG_REQUEST_RETURN_em] = {.name = "request_return",.val_em = UPDATE_STATE_NO_DATA_em},
    [UPDATE_FLAG_DELVIERY_SPECIAL_em] = {.name = "delivery_special",.val_em = UPDATE_STATE_NO_DATA_em},
    [UPDATE_FLAG_BEGIN_FLY_TASK_em] = {.name = "start_fly_task",.val_em = UPDATE_STATE_NO_DATA_em},
    [UPDATE_FLAG_FINISH_SPECIAL_em] = {.name = "finish_special",.val_em = UPDATE_STATE_NO_DATA_em},
};

/* 编译期校验：designated initializer 的最大下标 + 1 == 数组长度 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[UPDATE_STATE_NO_DATA_em]))

/* ── 边界保护 ────────────────────────────────────────────
 * 防御性编程：enum 值超出范围时不越界访问
 * 类似 Linux 内核中 WARN_ON_ONCE() 的做法
 */
static inline bool field_valid(enum update_flag_field_e field_uc)
{
    if ( (uint32_t)field_uc >= ARRAY_SIZE(s_fields)) {
        /* 枚举越界 — 可能是 enum 和 s_fields[] 不同步 */
        return false;
    }
    return true;
}

/* ── 公开 API ──────────────────────────────────────────── */

/*
 * update_flag_consume_uc - 读取标志位并自动清零
 * @field_uc: 标志位枚举值
 *
 * 返回当前值，调用后标志位归零。
 * 语义等同于 Linux 的 xchg() / test_and_clear_bit()。
 */
enum update_state_e update_flag_consume_uc(enum update_flag_field_e field_uc)
{
    if (!field_valid(field_uc))
        return UPDATE_STATE_NO_DATA_em;

    enum update_state_e val_em = s_fields[field_uc].val_em;
    s_fields[field_uc].val_em = UPDATE_STATE_NO_DATA_em;
    return val_em;
}

/*
 * update_flag_set_v - 设置标志位
 * @field_uc: 标志位枚举值
 * @val_uc:   要写入的值（通常为 1）
 *
 * 类似 Linux 的 set_bit() / WRITE_ONCE()。
 */
void update_flag_set_v(enum update_flag_field_e field_uc)
{
    if (!field_valid(field_uc))
        return;

    s_fields[field_uc].val_em = UPDATE_STATE_HAS_DATA_em;
}

// void update_flag_dump_v(void)
// {
//     for (uint32_t i = UPDATE_STATE_NO_DATA_em; i < ARRAY_SIZE(s_fields); i++) {
//         printf("[%2d] %-20s = %d\r\n",
//                (int)i, s_fields[i].name, s_fields[i].val_em);
//     }
// }
