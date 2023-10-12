
/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   main.c                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     2.0.0                                                        *
 *                                                                             *
 * [DATE]:        29/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]:                                                              *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                              INCLUDES
 *******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "APPLICATION/app.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

TaskHandle_t TaskHandle_ultrasonic;
TaskHandle_t TaskHandle_ldr       ;
TaskHandle_t TaskHandle_robot     ;
TaskHandle_t TaskHandle_lcd       ;
TaskHandle_t TaskHandle_temp      ;
TaskHandle_t TaskHandle_switches  ;

xSemaphoreHandle xBinarySemaphore_LDR     ;
xSemaphoreHandle xBinarySemaphore_obstacle;

xQueueHandle xQueue_LDR_Difference                   ;
xQueueHandle xQueue_LDR_Difference_LCD               ;
xQueueHandle xQueue_LDR_Difference_temperature_value ;

void main(void)
{
    /*initialize the system*/
    system_init();

    lcd_display_string("Auto Car RTOS");
    /*create the tasks*/
    xTaskCreate(switches_update    ,"switches_update"   , configMINIMAL_STACK_SIZE , NULL , 1 , &TaskHandle_switches  );
    xTaskCreate(ultrasonic_update  ,"ultrasonic_update" , configMINIMAL_STACK_SIZE , NULL , 4 , &TaskHandle_ultrasonic);
    xTaskCreate(ldr_update         ,"ldr_update"        , configMINIMAL_STACK_SIZE , NULL , 3 , &TaskHandle_ldr       );
    xTaskCreate(robot_state        ,"robot_state"       , configMINIMAL_STACK_SIZE , NULL , 2 , &TaskHandle_robot     );
    xTaskCreate(lcd_update         ,"lcd_update"        , configMINIMAL_STACK_SIZE , NULL , 1 , &TaskHandle_lcd       );
    xTaskCreate(temperature_update ,"temperature_update", configMINIMAL_STACK_SIZE , NULL , 1 , &TaskHandle_temp      );

    vSemaphoreCreateBinary(xBinarySemaphore_LDR);
    vSemaphoreCreateBinary(xBinarySemaphore_obstacle);
    xSemaphoreTake(xBinarySemaphore_LDR , 1);
    xSemaphoreTake(xBinarySemaphore_obstacle , 1);

    xQueue_LDR_Difference                   = xQueueCreate(1, sizeof(int32_t));
    xQueue_LDR_Difference_LCD               = xQueueCreate(1, sizeof(int32_t));
    xQueue_LDR_Difference_temperature_value = xQueueCreate(1, sizeof(int32_t));
    vTaskSuspend(TaskHandle_ultrasonic) ;
    vTaskSuspend(TaskHandle_ldr       ) ;
    vTaskSuspend(TaskHandle_robot     ) ;
    vTaskSuspend(TaskHandle_lcd       ) ;
    vTaskSuspend(TaskHandle_temp      ) ;
    vTaskStartScheduler();
    while (1) ;

}
