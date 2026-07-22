#include "ano_device_ground.h"//从飞控到地面站的所有数据
/*struct 
Animal_Report_Data_t->row_uc(Y)\col_uc(X)\count_per_type_auc[6]
gs_radar_pos_t->x_x100_s\y_x100_s\z_x100_s\freq_x10_s
gs_radar_speed_t->x_x100_s\y_x100_s\z_x100_s\freq_x10_s
这些参数对目标而言的
gs_jn_cam_t->cam_x_x100_s\cam_y_x100_s\state_uc\type_uc\id_s\freq_x10_s
gs_radar_yaw_t->yaw_s\freq_x10_s偏航角信息
gs_pwm_t->pwm_m1\pwm_m2\pwm_m3\pwm_m4
gs_batt_curr_height_process_t->voltage_100\current_100\alt_fu_ul\process_uc
控制指令传输gs_cmd_vel_t->rol\pit\thr\yaw_dps\vel_x\vel_y\vel_z\freq_x10_s
巡航速度配置gs_vel_fu_t->vel_x_s\vel_y_s\vel_z_s\freq_x10_s
飞控传回动物数据gs_animal_report_t->row_uc\col_uc\count_per_type_auc[6]
飞控运行状态gs_fc_status_t->phase_uc(当前飞行阶段)\pos_x_s(当前飞行位置)\pos_y_s
*void
vGround_DT_Data_Receive_Anl_Ano(uint8_t *pucdata,uint8_t uclen
*/
#include "driver_registry.h"
#include "ano_device_usart3.h"
#include "ano_base.h"
#include "ano_protocol.h"
#include "ano_ture.h"
#include "ano.h"
#include "SysConfig.h"
#include "types.h"
#include "Drv_Uart.h"
#include "ring_buffer.h"
#include "uart_base.h"
#include "uart_log.h"
#include "uart_true.h"
#include "uarts.h"
#include "mission_planner.h"
#include "route_planning.h"
#include "touch_uart.h"
/**
 * struct
 * ui_mode_t->eum
 * 预览模式UI_MODE_PREVIEW\单显只显示巡查UI_MODE_PATROL\单显只显示返航UI_MODE_RETURN
 * screen_state_t->bool start_fly_task_b飞行任务启动标志位
 * FlightPath->count(航点数量)\Point_index_t(这个里面有x and y)
 * Point_index_t->x,y
 * void
 *读取串口屏的发送信息 screen_uart_check_touch(void)
 * 飞行任务是否开始screen_begin_fly_b(void)
 * UI模式切换，清空屏幕旧路径，从mission层读取对应航线
 * 并下发到屏幕绘制screen_set_ui_mode(ui_mode_t mode)
 * dispath_line(const char *line)
 * 根据命令行前缀写一个命令字符串支持指令(zone:\request_route\onlu_path\only_return\start_flytask\stop_flytask)
 * parse_zone_command(*line)//禁飞区命令
 * 航线数据下发屏幕，screen_send_path(*prefix,FlightPath *fp)
 * covx(x,y)返回数组
 * FlightPath
 */
#include "Ano_Scheduler.h"