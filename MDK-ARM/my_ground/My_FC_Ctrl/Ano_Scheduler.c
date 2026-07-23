#include "Ano_Scheduler.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "driver_registry.h"
#include "ano_device_ground.h"
#include "Drv_Uart.h"
#include "ano_device_usart3.h"
#include "uart_log.h"
#include "uarts.h"
#include "mission_planner.h"
#include "touch_uart.h"
#include "Report/report.h"
#include "update.h"
#include "ano.h"

extern osThreadId_t UART_TouchHandle;
extern osThreadId_t Ground_UARTHandle;

void APPTask_LX(void *argument)
{
  driver_init_all();
#if POINT_DEBUG
  struct Point_3D_t temp = {.x_s = 120,.y_s = 150,.z_s = 180,.yaw_s  = 180};

  point_3d_add_b(g_partrol_point_3d_pst, &temp);
  temp.yaw_s  = 120;
  point_3d_add_b(g_partrol_point_3d_pst, &temp);
#endif
  static uint32_t s_last_tick_pul[2] = {0};

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    uint32_t current_tick_ul =  xTaskGetTickCount();
    DrvUartDataCheck();
    if (current_tick_ul - s_last_tick_pul[0] >= 10)
    {
        s_last_tick_pul[0] = current_tick_ul;
        mission_planner_tick();
    }

    if (current_tick_ul - s_last_tick_pul[1] >= 1000)
    {
        s_last_tick_pul[1] = current_tick_ul;
#if POINT_DEBUG
        static struct delivery_t s_temp_st = {0};
        
        struct Point_3D_t point_3d_st = {0};
        if(point_3d_take_uc(g_partrol_point_3d_pst,&point_3d_st) == 0)
        {
          uart_printf_v(pstbase_screen_uart,0,"%d,%d,%d,%d\r\n",point_3d_st.x_s,point_3d_st.y_s,point_3d_st.z_s,point_3d_st.yaw_s);
        }
#endif
        if (request_route_b() == true)
        {
            static uint8_t phase_uc = 0;
            switch (phase_uc)
            {
            case 0:
                if (point_3d_is_empty_b(g_partrol_point_3d_pst) == true)
                {
                    phase_uc = 1;
                }
                else
                {
                    vano_WTS_set(pstAnobase_Ground, 0x16, 1);
                }
                break;
            case 1: {
                if (point_3d_is_empty_b(g_return_point_3d_pst) == true)
                {
                    phase_uc = 0;
                }
                else
                {
                    vano_WTS_set(pstAnobase_Ground, 0x17, 1);
                }
            }
            break;
            default:
                break;
            }
          }

    }
    
    xTaskNotifyGive(Ground_UARTHandle);
    xTaskNotifyGive(UART_TouchHandle);

  }

}


 void Ground_DT_UART(void *argument)
{

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    vGround_Data_Exchange_Task_Ano();
    vUsart3_Data_Exchange_Task_Ano();
  }

}


void UART_DT_Touch(void *argument)
{

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    screen_send_delivery();
  }

}


