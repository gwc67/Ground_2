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
extern osThreadId_t UART_TouchHandle;
extern osThreadId_t Ground_UARTHandle;

void APPTask_LX(void *argument)
{
  driver_init_all();

  struct delivery_t a = { .x_s = 120,.y_s = 60,.z_s = 80,.type_uc = 0,.position_uc = 0};
  struct delivery_t b = { .x_s = 180,.y_s = 70,.z_s = 80,.type_uc = 1,.position_uc = 1};

  delivery_add_b(&a);
  delivery_add_b(&b);
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

    if (current_tick_ul - s_last_tick_pul[1] >= 100)
    {
        s_last_tick_pul[1] = current_tick_ul;
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


