/**
 * touch_uart.c — 屏幕串口（USART2）协议层
 *
 * 职责（纯屏幕协议，不含任务逻辑）：
 * 1. 行组装 + 命令解析
 * 2. 屏幕协议发送（path: 分包、ui_mode: 切换）
 * 3. FC 相位轮询 → 驱动屏幕 UI 更新
 *
 * 依赖方向：touch_uart → mission（单向）
 *           mission 不知道 touch_uart 的存在
 */
// 本文件自身的头文件，声明了对外接口（screen_uart_check_touch / screen_set_ui_mode 等）
#include "touch_uart.h"//自己的头文件，方便自引用
// 串口底层基类：定义 stUartBase 句柄、读字节函数 uc_uart_read_ucbyte 等
#include "uart_base.h"
// 串口日志打印封装（uart_printf_v 等格式化发送）
#include "uart_log.h"
// 串口实例表：pstbase_screen_uart 即本文件操作的“屏幕串口”句柄（USART2）
#include "uarts.h"
// 任务规划层接口：禁飞区设置 / 路线请求 / 路线拷贝等（touch_uart 单向依赖 mission）
#include "Mission/mission_planner.h"
// 共享数据类型：struct Point_index_t（网格坐标）、struct FlightPath（航线）等
#include "Common/types.h"
// 标准库：strncpy / strcmp / strncmp 字符串比较
#include <string.h>
// 标准库：strtol（字符串→整数），用于解析命令里的坐标数值
#include <stdlib.h>
// ANO 协议设备层（飞控通信），本文件内仅在头文件依赖链中被引用
#include "ano_device_ground.h"
#include "point_3d.h"
#include "Ano_Scheduler.h"
#include "Report/report.h"
#if PYTHON_DEBUG
/* ================================================================
 *              Python 上位机模式（PYTHON_DEBUG = 1）
 * ================================================================ */

// 一行命令的最大缓冲长度（含结尾 '\0'）
#define LINE_BUF_SIZE  128
// 行缓冲区：正在接收的一行命令先存到这里，遇到换行再解析
static char s_line_buf[LINE_BUF_SIZE];
// 当前缓冲区写入位置（已写入的字节数），一行结束（\n/\r）后归零
static uint8_t s_line_pos = 0;

/**
 * parse_map_command - 解析 "MAP:" 命令
 *
 * 格式：MAP:x1,y1,x2,y2,...（成对的坐标，最多 3 个禁飞区 = 6 个数值）
 * 解析后调用 mission 层设置禁飞区
 */
// 参数 line 指向 "MAP:..." 整行命令字符串
static void parse_map_command(const char *line)
{
    // p 跳过前缀 "MAP:"（4 个字符），指向第一个数字
    const char *p = line + 4;
    // zones：禁飞区坐标数组（最多 3 个点），解析完传给 mission 层
    struct Point zones[3];
    // count：实际解析出的数值个数（每 2 个数值 = 1 个禁飞区点）
    uint8_t count = 0;
    // coords：暂存解析出的 6 个整数（x1,y1,x2,y2,x3,y3）
    int coords[6];

    // 最多解析 6 个数值
    for (int i = 0; i < 6; i++) {
        char *end;
        // strtol：从 p 起按 10 进制解析一个整数，end 指向数字之后第一个非数字字符
        long val = strtol(p, &end, 10);
        // end == p 表示没解析到任何数字 → 提前结束
        if (end == p) break;
        // 把解析到的数值存入 coords
        coords[i] = (int)val;
        // p 移动到本次解析结束处，准备解析下一个数字
        p = end;
        // 数字之间用逗号分隔，跳过一个逗号（若有）
        if (*p == ',') p++;
        // 已成功解析的数值计数 +1
        count++;
    }

    // 至少解析到 2 个数值（即至少 1 个完整点）才设置禁飞区
    if (count >= 2) {
        // 点数 = 数值数 / 2（每点 2 个数值 x,y）
        uint8_t zone_count = count / 2;
        // 把 coords 里成对的数值转成 zones 点结构
        for (uint8_t i = 0; i < zone_count; i++) {
            zones[i].x = (uint8_t)coords[i * 2];      // 第 i 个点的 x
            zones[i].y = (uint8_t)coords[i * 2 + 1]; // 第 i 个点的 y
        }
        // 调用 mission 层：覆盖式写入这批禁飞区
        mission_set_no_fly_zones(zone_count, zones);
    }
}

/**
 * dispatch_line - 命令分发
 *
 * 根据一行命令的前缀，路由到对应的处理逻辑
 */
// 参数 line 指向一行完整命令（已去换行、已 '\0' 结尾）
static void dispatch_line(const char *line)
{
    // "MAP:" 前缀 → 解析禁飞区地图命令
    if (strncmp(line, "MAP:", 4) == 0) {
        parse_map_command(line);
    } else if (strcmp(line, "REQUEST_PATH") == 0) {
        // "REQUEST_PATH" → 请求 mission 层计算路线（计算+缓存双路线）
        mission_handle_request_route();
    }
}

/**
 * screen_uart_check_python - Python 模式下的串口轮询入口
 *
 * 在主循环 / DrvUartDataCheck 中被调用，读取屏幕串口字节并组装成行
 */
// 无参，读全局的屏幕串口句柄 pstbase_screen_uart
void screen_uart_check_python(void)
{
    uint8_t byte;  // 每次读到的一个字节

    // 循环读取：uc_uart_read_ucbyte 返回 0 表示成功读到字节，!=0 表示缓冲区空
    while (uc_uart_read_ucbyte(pstbase_screen_uart, &byte) == 0) {
        // 遇到换行或回车 → 一行结束
        if (byte == '\n' || byte == '\r') {
            // 只有缓冲区非空（确实收到内容）才解析
            if (s_line_pos > 0) {
                // 在缓冲区末尾补 '\0'，凑成 C 字符串
                s_line_buf[s_line_pos] = '\0';
                // 解析并分发这一行
                dispatch_line(s_line_buf);
                // 复位写入位置，准备接收下一行
                s_line_pos = 0;
            }
            continue;  // 换行符本身不入缓冲区，直接跳过
        }
        // 普通字符：若缓冲区未满则写入
        if (s_line_pos < LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_pos++] = (char)byte;
        } else {
            // 缓冲区已满（超长行）：丢弃当前行，从 0 重新开始，避免越界
            s_line_pos = 0;
        }
    }
}


#else
/* ================================================================
 *              真实串口屏模式（PYTHON_DEBUG = 0）
 *              陶晶驰 TJC/Nextion 协议
 * ================================================================ */
#define LINE_BUF_SIZE  128
#define POINTS_PER_LINE  10
static char s_line_buf[LINE_BUF_SIZE];
struct screen_state_t
{
    bool start_fly_task_b;
    bool request_route_b;
};
static struct screen_state_t s_screen_state_st;



int covx(int x,int y)
{
    int sum;
    sum=(6-y)*9+x;
    return sum;
}
static void screen_send_path(const char *prefix, const struct FlightPath *fp)
{
    if (fp->count == 0) {
        uart_printf_v(pstbase_screen_uart, 0, "t2.txt=%s\xff\xff\xff", prefix);
        return;
    }
    
    if(strcmp(prefix,"返航模式") == 0)
    {
        uart_printf_v(pstbase_screen_uart,0,"tip1=5\xff\xff\xff");
    //第一个航点是起点，但是我的地面站显示的时候是手动点击的，所以我直接从i=1开始就可以了
        for (uint8_t i = 0; i < fp->count; i++)
    {
        uart_printf_v(pstbase_screen_uart, 0,"source=%d\xff\xff\xff",
                      covx(fp->points[i].x, fp->points[i].y));
        uart_printf_v(pstbase_screen_uart,0,"click m1,1\xff\xff\xff");
    }
    }else if(strcmp(prefix,"巡查模式") == 0)
    {
        uart_printf_v(pstbase_screen_uart,0,"tip1=4\xff\xff\xff");
        for (uint8_t i = 0; i < fp->count; i++)
    {
        uart_printf_v(pstbase_screen_uart, 0,"source=%d\xff\xff\xff",
                      covx(fp->points[i].x, fp->points[i].y));
        uart_printf_v(pstbase_screen_uart,0,"click m0,1\xff\xff\xff");
    }
    }
    uart_printf_v(pstbase_screen_uart, 0, "click t2,1\xff\xff\xff");
    uart_printf_v(pstbase_screen_uart,0,"click t2,0\xff\xff\xff");
}
void screen_set_ui_mode(ui_mode_t mode)
{
    uart_printf_v(pstbase_screen_uart, 0, "click b1,0\xff\xff\xff");
    struct FlightPath path_st = {0};
    struct FlightPath return_st = {0};
    switch (mode) {
    case UI_MODE_PREVIEW:
    {
        // 预览模式：同时显示巡查 + 返航两条路线
        mission_copy_patrol_screen_path_v(&path_st);
        screen_send_path("巡查模式",&path_st);      // 发巡查路径
        mission_copy_return_screen_path_v(&return_st);
        screen_send_path("返航模式",&return_st);  // 发返航路径
    }
    break;
    case UI_MODE_PATROL:
    {
        // 仅巡查模式：只发巡查路径
        mission_copy_patrol_screen_path_v(&path_st);
        screen_send_path("巡查模式",&path_st);
    }
    break;
    case UI_MODE_RETURN:
    {
        // 仅返航模式：只发返航路径
        mission_copy_return_screen_path_v(&return_st);
        screen_send_path("返航模式",&return_st);
    }
    break;
    default: return;
    }
}
//处理串口屏的三个禁飞区的设置
//static void parse_zone_command(const char *line)
//{
//    const char *p = line + 5;
//    struct Point_index_t zones[3];
//    uint8_t count = 0;
//    int coords[6];
//    for (int i = 0; i < 6; i++) {
//        char *end;
//        while (*p && *p != '-' && *p != '+' &&
//               (*p < 0 || *p > 9))
//            p++;
//        if (*p == '\0')
//            break;
//        long val = strtol(p, &end, 10);
//        if (end == p)
//            break;
//        coords[i] = (int)val;
//        p = end;
//        count++;
//    }
//    if (count >= 2) {
//        uint8_t zone_count = count / 2;
//        for (uint8_t i = 0; i < zone_count; i++) {
//            zones[i].x = (uint8_t)coords[i * 2];
//            zones[i].y = (uint8_t)coords[i * 2 + 1]; 
//        }
//        mission_reset_no_fly_zones_v();
//        mission_set_no_fly_zones(zone_count, zones);
//    }
//}

//这个应该是轮询发送的才对
void screen_send_delivery(void)
{
    static uint8_t s_has_sent_index = 0;
    uint8_t delivery_index_uc = delivery_get_cur_index();
    
    if (s_has_sent_index != delivery_index_uc)
    {
                                   
        struct delivery_t delivery_st = {0};
        delivery_copy_by_index_b(s_has_sent_index,&delivery_st);
        
        s_has_sent_index++;                         //只能加加，直到已经发送过的和现在的索引的一模一样
        uart_printf_v(pstbase_screen_uart,0,"result.data0.insert(\"%d\")\xff\xff\xff",delivery_st.position_uc);        //A1 ~ A6 B1 ~ B6 C1 ~ C6 D1 ~ D6
        uart_printf_v(pstbase_screen_uart,0,"result.data4.insert(\"%d\")\xff\xff\xff",delivery_st.type_uc);                //1 ~ 24
        uart_printf_v(pstbase_screen_uart,0,"result.data1.insert(\"%d\")\xff\xff\xff",delivery_st.x_s);
        uart_printf_v(pstbase_screen_uart,0,"result.data2.insert(\"%d\")\xff\xff\xff",delivery_st.y_s);
        uart_printf_v(pstbase_screen_uart,0,"result.data3.insert(\"%d\")\xff\xff\xff",delivery_st.z_s);
    }
    
    
}

// static void parse_zone_command(const char *line)
// {
//     const uint8_t *p = (const uint8_t *)line + 5;  // 跳过 "zone:" 5字节前缀
//     struct Point_index_t zones[3];
//     uint8_t count = 0;

//     // 循环解析每组 (x,y)，最多支持3组禁飞区
//     while (*p != '\0' && count < 3)
//     {
//         if (*p == '(')
//         {
//             p++;
//             uint8_t x = *p++;
//             p++;
//             uint8_t y = *p++;
//             p++;
//             if (x < 9 && y < 7)
//             {
//                 zones[count].x = x;
//                 zones[count].y = y;
//                 count++;
//             }
//         }
//         else
//         {
//             p++;
//         }
//     }
//     mission_reset_no_fly_zones_v();
//     if (count > 0)
//     {
//         mission_set_no_fly_zones(count, zones);
//     }
// }


static uint8_t str_to_int16_uc(const char *str, int16_t *array_ps, uint8_t arr_size_uc);
static void parse_zone_command(const char *line);
static void parse_delivery_command(const char *line);

static void dispatch_line(const char *line)
{
    if (strncmp(line, "zone:", 5) == 0) {
        line += strlen("zone:");
        parse_zone_command(line);
    } 
    else if (strncmp(line,"delivery:",9) == 0)
    {
        line += strlen("delivery:");
        parse_delivery_command(line);
    }
    
    else if (strcmp(line, "request_route") == 0) {
        if (mission_handle_request_route()) {
            s_screen_state_st.request_route_b = true;
            screen_set_ui_mode(UI_MODE_PREVIEW);
        }
    }
    // "only_path" → 只显示巡查路线
    else if (strcmp(line,"only_path") == 0)
    {
        screen_set_ui_mode(UI_MODE_PATROL);
    }
    // "only_return" → 只显示返航路线
    else if (strcmp(line,"only_return") == 0)
    {
        screen_set_ui_mode(UI_MODE_RETURN);
    }
    // "start_flytask" → 置位“开始飞行”标志
    else if (strcmp(line,"start_flytask") == 0)
    {
       if (s_screen_state_st.request_route_b)
        {
#if COM_DEBUG
            uart_printf_v(pstbase_screen_uart, 0, "start_flytask success !\r\n");
#endif
            s_screen_state_st.start_fly_task_b = true;
        }
        else
        {
#if COM_DEBUG
            uart_printf_v(pstbase_screen_uart, 0, "start_flytask fail !\r\n");
#endif
            s_screen_state_st.start_fly_task_b = false;
        }
    }
    // "stop_flytask" → 清除“开始飞行”标志
    else if(strcmp(line,"stop_flytask") == 0)
    {
        s_screen_state_st.start_fly_task_b = false;
    }
}



/**
 * 解析串口屏命令（DrvUartDataCheck 中调用）
 */
//静态变量，如何保证lie_pos_uc的值是正确的？
//也就是期望串口屏幕数出
// 屏幕串口轮询入口：逐字节读取并组装成行，遇到换行就分发
void screen_uart_check_touch(void)
{
    uint8_t byte;         // 每次读到的一个字节
    uint8_t lie_pos_uc = 0;// 局部游标：本行已写入缓冲区的字节数（注意：非 static，每次调用从 0 开始）
    // 循环读取：返回 0 表示成功读到字节，!=0 表示缓冲区空
    while (uc_uart_read_ucbyte(pstbase_screen_uart, &byte) == 0) {
        // 遇到换行或回车 → 一行结束
        if (byte == '\n' || byte == '\r') {
            // 只有缓冲区非空才解析
            if (lie_pos_uc > 0) {
                // 在末尾补 '\0'，凑成 C 字符串
                s_line_buf[lie_pos_uc] = '\0';
                //此时这个确实是s_line_buf 的最后一个值
                // 解析并分发这一行
                dispatch_line(s_line_buf);
                lie_pos_uc = 0;
            }
            continue;  // 换行符本身不入缓冲区
        }
        if (lie_pos_uc < LINE_BUF_SIZE - 1) {
            s_line_buf[lie_pos_uc++] = (char)byte;
        } else {
            lie_pos_uc = 0;
        }
    }
}
/**
 * FC 相位轮询 → 屏幕 UI 更新（DrvUartDataCheck 中调用）
 *
 * 职责：检测 FC 相位变化 → 发送 ui_mode: 指令到屏幕
 * 与 screen_uart_check_touch() 分离：
 *   check_touch = 被动收命令
 *   check_phase = 主动查状态
 */
// 查询“是否已收到开始飞行任务指令”，供主循环 / 任务层判断是否启动飞行
 bool screen_begin_fly_b(void)
 {
    return s_screen_state_st.start_fly_task_b;
 }

static uint8_t str_to_int16_uc(const char *str, int16_t *array_ps, uint8_t arr_size_uc)
{
    if (str == NULL || array_ps == NULL || arr_size_uc == 0) {
        return 0;
    }

    uint8_t count = 0;
    
    while (*str != '\0' && count < arr_size_uc) 
    {

        while (*str != '\0' && *str != '-' && *str != '+' && (*str < '0' || *str > '9')) {
            str++;
        }
        if (*str == '\0') break;


        int32_t sign = 1;
        if (*str == '-') { sign = -1; str++; }
        else if (*str == '+') { str++; }


        if (*str < '0' || *str > '9') continue;

        int32_t val = 0;
        while (*str >= '0' && *str <= '9') {
            val = val * 10 + (*str - '0');
            str++;
        }
        val *= sign;

        if (val > INT16_MAX) val = INT16_MAX;       // 32767
        else if (val < INT16_MIN) val = INT16_MIN;  // -32768

        array_ps[count++] = (int16_t)val;
    }
    return count;
}


 static void parse_zone_command(const char *line)
{
    struct Point_index_t zones[3];

    int16_t array_ps[20] = {0};

    uint8_t data_num =  str_to_int16_uc(line,array_ps,sizeof(array_ps) / sizeof(array_ps[0]));
    

    for (uint8_t i = 0; i < data_num ; i+=2)
    {
        zones[i/2].x = (uint8_t)array_ps[i];
        zones[i/2].y = (uint8_t)array_ps[i+1];
#if TOUCH_UART_DEBUG
    uart_printf_v(pstbase_screen_uart,0,"set zone: x:%d , y:%d\r\n",zones[i/2].x,zones[i/2].y);
#endif
    }
    

    
    mission_reset_no_fly_zones_v();

    if (data_num >=2)
    {
        mission_set_no_fly_zones(data_num/2, zones);
    }
    
}
 

static void parse_delivery_command(const char *line)
{

    uint8_t count_uc = 0;
    int16_t array_ps[20] = {0};
    struct Point_3D_t temp_st = {0};

    uint8_t data_num =  str_to_int16_uc(line,array_ps,sizeof(array_ps) / sizeof(array_ps[0]));


    temp_st.x_s = array_ps[count_uc++];
    temp_st.y_s = array_ps[count_uc++];
    temp_st.z_s = array_ps[count_uc++];


#if TOUCH_UART_DEBUG
    uart_printf_v(pstbase_screen_uart,0,"target_pos:x:%d,y:%d,z:%d\r\n",temp_st.x_s,temp_st.y_s,temp_st.z_s);
#endif
}

 
#endif

