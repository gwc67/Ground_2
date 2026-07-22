# AutoInit — 驱动自动初始化机制

## 概述

通过编译器段属性和链接器行为，实现类似 Linux kernel 的 `initcall` 机制：驱动只需用 `DRIVER_INIT` 宏注册，无需手动调用，`main()` 中 `driver_init_all()` 自动遍历执行所有注册的初始化函数。

## 文件清单

| 文件 | 职责 |
|------|------|
| `driver_registry.h` | `driver_entry_t` 类型定义、`DRIVER_INIT` 注册宏、`driver_init_all()` 遍历函数 |
| `driver_registry.c` | 段首/段尾标记（`.dvr.a` / `.dvr.z`） |

## API

### 注册驱动

```c
#include "driver_registry.h"

DRIVER_INIT(board_init);       // 注册 board_init 为自动初始化函数
DRIVER_INIT(uart_board_init);  // 注册 uart_board_init
```

支持在任意 `.c` 文件中注册任意数量的初始化函数。

### 触发执行

```c
// main.c 中调用一次，遍历所有注册的初始化函数
driver_init_all();
```

需要在 HAL 外设初始化（`MX_xxx_Init()`）完成之后调用。

## 原理

### 三段式段名

利用 ASCII 字母序保证内存布局顺序：

```
段名      ASCII 码   用途
.dvr.a   0x61       段首标记（最低地址）
.dvr.b   0x62       注册条目（中间，任意多个）
.dvr.z   0x7A       段尾标记（最高地址）
```

链接器按段名字母序排列各段，`a < b < z` 保证 `.dvr.a` 在最低地址、`.dvr.z` 在最高地址、`.dvr.b` 在中间。

### 链接器合并同名段

所有 `.o` 中同名段（`.dvr.b`）被合并成一个连续块：

```
内存布局：
低地址  .dvr.a  → __driver_init_begin    {NULL, NULL}      ← 跳过
        .dvr.b  → __driver_board_init    {board_init, ...}  ← 执行
                 → __driver_uart_init    {uart_init, ...}   ← 执行
                 → ...                   （可以有 N 个条目）
高地址  .dvr.z  → __driver_init_end      {NULL, NULL}      ← 停下
```

### 遍历逻辑

```c
const driver_entry_t *p = __driver_init_begin + 1;  // 跳过段首标记
const driver_entry_t *end = __driver_init_end;       // 段尾标记

while (p < end) {
    if (p->init_fn) {
        p->init_fn();   // 调用初始化函数
    }
    p++;
}
```

### 同名段（`.dvr.b`）内部的顺序

同名段之间的顺序由链接器处理目标文件的次序决定。在 EIDE 中，由 `eide.yml` 的 `srcDirs` 列表顺序决定：

```yaml
srcDirs:
  - ../DriversBsp        # 先处理 → .dvr.b 条目排在前面
  - ../FcSrc
  - DriversMcu           # 后处理 → .dvr.b 条目排在后面
```

如果多个初始化函数之间存在依赖关系（如 A 必须在 B 之前执行），且它们不在同一个目录下，可以通过不同的段名实现更细粒度的控制：

```c
#define DRIVER_INIT_1(fn) \
    const driver_entry_t __driver_##fn \
        __attribute__((used, section(".dvr.b1"))) = { .init_fn = fn, .name = #fn }

#define DRIVER_INIT_2(fn) \
    const driver_entry_t __driver_##fn \
        __attribute__((used, section(".dvr.b2"))) = { .init_fn = fn, .name = #fn }
```

`b1` < `b2`，链接器保证 `.dvr.b1` 在 `.dvr.b2` 之前。

### 注意事项

1. 该方法不依赖自定义 scatter 文件，对 ARM 链接器（armlink）和 GCC 链接器均适用
2. 不能使用 `Image$$section$$Base/Limit` 符号——ARM 链接器只对 execution region 自动生成，不对 input section 生成
3. 不能使用 `__start_`/`__stop_` 符号——这是 GCC 特性，ARM 链接器不支持
4. `DRIVER_INIT` 注册的变量不能加 `static`——否则被链接器优化丢弃（即使加了 `__attribute__((used))`）
