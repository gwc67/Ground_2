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
#include "update.h"
#include "ano.h"
#include "map/point_2d.h"

#define LINE_BUF_SIZE               128
#define POINTS_PER_LINE             10

#define CLEAR_POINT_TX                 0x20     
 


static char s_line_buf[LINE_BUF_SIZE];
struct screen_state_t
{
    bool request_route_b;
};
static struct screen_state_t s_screen_state_st;



int covx(int x,int y)
{
    int sum;
    sum=(6-y)*9+x;
    return sum;
}

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

#if TOUCH_UART_DEBUG

        uart_printf_v(pstbase_screen_uart,0,"\r\npoistion:%d\r\n",delivery_st.position_uc);        //A1 ~ A6 B1 ~ B6 C1 ~ C6 D1 ~ D6
        uart_printf_v(pstbase_screen_uart,0,"x:%d\r\n",delivery_st.x_s);
        uart_printf_v(pstbase_screen_uart,0,"y:%d\r\n",delivery_st.y_s);
        uart_printf_v(pstbase_screen_uart,0,"z:%d\r\n",delivery_st.z_s);
        uart_printf_v(pstbase_screen_uart,0,"type:%d\r\n",delivery_st.type_uc);    
#else
        uart_printf_v(pstbase_screen_uart,0,"result.data0.insert(\"%d\")\xff\xff\xff",delivery_st.position_uc);        //A1 ~ A6 B1 ~ B6 C1 ~ C6 D1 ~ D6
        uart_printf_v(pstbase_screen_uart,0,"result.data1.insert(\"%d\")\xff\xff\xff",delivery_st.x_s);
        uart_printf_v(pstbase_screen_uart,0,"result.data2.insert(\"%d\")\xff\xff\xff",delivery_st.y_s);
        uart_printf_v(pstbase_screen_uart,0,"result.data3.insert(\"%d\")\xff\xff\xff",delivery_st.z_s);
        uart_printf_v(pstbase_screen_uart,0,"result.data4.insert(\"%d\")\xff\xff\xff",delivery_st.type_uc);                //1 ~ 24
#endif
    }
    
    if (update_flag_consume_uc(UPDATE_FLAG_DELVIERY_SPECIAL_em))            //类似刚刚手动切换成type_一样，这是只是使用无人机切换罢了
    {
        struct delivery_t delivery_st = {0};
        delivery_copy_special(&delivery_st);
#if TOUCH_UART_DEBUG
        uart_printf_v(pstbase_screen_uart,0,"target_type_uc:%d",delivery_st.type_uc);
#else   
        uart_printf_v(pstbase_screen_uart,0,"result.data4.insert(\"%d\")\xff\xff\xff",delivery_st.type_uc);         //货物编号显示
#endif
    }
    
    
}





static uint8_t str_to_int16_uc(const char *str, int16_t *array_ps, uint8_t arr_size_uc);
static void parse_zone_command(const char *line);
static void parse_delivery_command(const char *line);
static void dispatch_line(const char *line);
 static void screen_send_path(const char *prefix, const struct Point_map_t *fp);
void screen_set_ui_mode(ui_mode_t mode);

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
            update_flag_set_v(UPDATE_FLAG_BEGIN_FLY_TASK_em);
            vano_WTS_set(pstAnobase_Ground,CLEAR_POINT_TX,1);
        }
        else
        {
#if COM_DEBUG
            uart_printf_v(pstbase_screen_uart, 0, "start_flytask fail !\r\n");
#endif
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
    struct delivery_t temp_st = {0};

    uint8_t data_num =  str_to_int16_uc(line,array_ps,sizeof(array_ps) / sizeof(array_ps[0]));


    temp_st.type_uc = (uint8_t)array_ps[count_uc++];


#if TOUCH_UART_DEBUG
    uart_printf_v(pstbase_screen_uart,0,"type_uc:%d\r\n",temp_st.type_uc);

    //调试阶段，可以自己设置，之后得关掉
    //根据设置的type_uc = 1, = 2 算出，并打印出此时的航点
    delivery_set_special(&temp_st);
#endif    
}

 static void screen_send_path(const char *prefix, const struct Point_map_t *fp)
{
#if TOUCH_UART_DEBUG
    if (fp->count_uc == 0)
    {
        uart_printf_v(pstbase_screen_uart, 0, "mode=%s\r\n", prefix);
        return;
    }
    uart_printf_v(pstbase_screen_uart, 0, "clear\r\n");

    uart_printf_v(pstbase_screen_uart,0,"%s_start\r\n",prefix);

    for (uint8_t i = 0; i < fp->count_uc; i++)
    {
        uart_printf_v(pstbase_screen_uart, 0, "(%d,%d)\r\n",
                      fp->point_mat_pst[i].x_c, fp->point_mat_pst[i].y_c);
    }
    uart_printf_v(pstbase_screen_uart,0,"%s_end\r\n",prefix);

#else
    if (fp->count_uc == 0) {
        uart_printf_v(pstbase_screen_uart, 0, "t2.txt=%s\xff\xff\xff", prefix);
        return;
    }
    
    if(strcmp(prefix,"返航模式") == 0)
    {
        uart_printf_v(pstbase_screen_uart,0,"tip1=5\xff\xff\xff");
    //第一个航点是起点，但是我的地面站显示的时候是手动点击的，所以我直接从i=1开始就可以了
        for (uint8_t i = 0; i < fp->count_uc; i++)
    {
        uart_printf_v(pstbase_screen_uart, 0,"source=%d\xff\xff\xff",
                      covx(fp->point_mat_pst[i].x_c, fp->point_mat_pst[i].y_c));
        uart_printf_v(pstbase_screen_uart,0,"click m1,1\xff\xff\xff");
    }
    }else if(strcmp(prefix,"巡查模式") == 0)
    {
        uart_printf_v(pstbase_screen_uart,0,"tip1=4\xff\xff\xff");
        for (uint8_t i = 0; i < fp->count_uc; i++)
    {
        uart_printf_v(pstbase_screen_uart, 0,"source=%d\xff\xff\xff",
                      covx(fp->point_mat_pst[i].x_c, fp->point_mat_pst[i].y_c));
        uart_printf_v(pstbase_screen_uart,0,"click m0,1\xff\xff\xff");
    }
    }
    uart_printf_v(pstbase_screen_uart, 0, "click t2,1\xff\xff\xff");
    uart_printf_v(pstbase_screen_uart,0,"click t2,0\xff\xff\xff");
#endif
}


void screen_set_ui_mode(ui_mode_t mode)
{
#if TOUCH_UART_DEBUG
    uart_printf_v(pstbase_screen_uart, 0, "screen_mode\r\n");
#else
    uart_printf_v(pstbase_screen_uart, 0, "click b1,0\xff\xff\xff");
#endif
    struct Point_map_t path_st = {0};
    struct Point_map_t return_st = {0};
    switch (mode) {
    case UI_MODE_PREVIEW:
    {
        // 预览模式：同时显示巡查 + 返航两条路线
        point_2d_patrol_take_v(&path_st);
        screen_send_path("巡查模式",&path_st);      // 发巡查路径
        // point_patrol_take_v(&return_st);
        // screen_send_path("返航模式",&return_st);  // 发返航路径
    }
    break;
    case UI_MODE_PATROL:
    {
        // 仅巡查模式：只发巡查路径
        // mission_copy_patrol_screen_path_v(&path_st);
        point_2d_patrol_take_v(&path_st);
        screen_send_path("巡查模式",&path_st);
    }
    break;
    case UI_MODE_RETURN:
    {
        // 仅返航模式：只发返航路径
        // mission_copy_return_screen_path_v(&return_st);
        point_2d_patrol_take_v(&path_st);
        screen_send_path("返航模式",&return_st);
    }
    break;
    default: return;
    }
}

bool request_route_b(void)
{
    return s_screen_state_st.request_route_b;
}

