#include "driver_registry.h"
#include "main.h"
// 段首标记 — .dvr.a 字母序最小，保证排在最前面
const driver_entry_t __driver_init_begin[]
    __attribute__((section(".dvr.a"), used)) = {{NULL, NULL}};

// 段尾标记 — .dvr.z 字母序最大，保证排在最后面
const driver_entry_t __driver_init_end[]
    __attribute__((section(".dvr.z"), used)) = {{NULL, NULL}};
