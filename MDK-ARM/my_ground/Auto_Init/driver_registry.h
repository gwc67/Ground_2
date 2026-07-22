#ifndef __DRIVER_REGISTRY_H
#define __DRIVER_REGISTRY_H

typedef void (*driver_init_func_t)(void);

typedef struct {
    driver_init_func_t init_fn;   // 初始化函数指针
    const char *name;             // 可选：驱动名（调试用）
} driver_entry_t;



// 段名按 ASCII 字母序排列（不使用 Image$$ 符号）：
//   .dvr.a  → 段首标记（最低地址）
//   .dvr.b  → 注册条目（中间）
//   .dvr.z  → 段尾标记（最高地址）
//
// 自动生成的 scatter 用 .ANY (+RO) 匹配这些段，
// 链接器按段名字母序放置，保证 a < b < z 的地址顺序
//   如果你确实需要严格顺序，拆分段名： //NB

  /* 最先执行 */
  #define DRIVER_INIT_1(fn) \
      const driver_entry_t __driver_##fn \
          __attribute__((used, section(".dvr.b1"))) = { .init_fn = fn, .name = #fn }

  /* 其次执行 */
  #define DRIVER_INIT_2(fn) \
      const driver_entry_t __driver_##fn \
          __attribute__((used, section(".dvr.b2"))) = { .init_fn = fn, .name = #fn }

#define DRIVER_INIT(fn) \
    const driver_entry_t __driver_##fn \
        __attribute__((used, section(".dvr.b3"))) = { .init_fn = fn, .name = #fn }
// ● 对于同名段（都是 .dvr.b），顺序由链接器处理目标文件的顺序决定。在 EIDE 中，这个顺序就是你 eide.yml 里 srcDirs 的排列顺序：
   
//   srcDirs:
//     - ../DriversBsp        # ← 第 1 优先
//     - ../FcSrc             # ← 第 2 优先
//     - DriversMcu           # ← 第 3 优先  ← boart_uart_init.c 在这里
//     - ../Middlewares
//     - ../My_FcSrc

// 遍历并调用所有注册的初始化函数
static inline void driver_init_all(void)
{
    extern const driver_entry_t __driver_init_begin[];
    extern const driver_entry_t __driver_init_end[];

    const driver_entry_t *p = __driver_init_begin + 1;  // 跳过段首标记
    const driver_entry_t *end = __driver_init_end;

    while (p < end) {
        if (p->init_fn) {
            p->init_fn();
        }
        p++;
    }
}

#endif
