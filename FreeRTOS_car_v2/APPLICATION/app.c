/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   app.c                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: This file contain the hardware connection of project         *
 *                                                                             *
 *******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app.h"

#define LDR_SET_POINT_POSTIVE 350


extern TaskHandle_t TaskHandle_ultrasonic;
extern TaskHandle_t TaskHandle_ldr       ;
extern TaskHandle_t TaskHandle_robot     ;
extern TaskHandle_t TaskHandle_lcd       ;
extern TaskHandle_t TaskHandle_temp      ;

/*****************************************************/
int LDR_Difference = 0;    //Q
int LDR_Difference_LCD = 0;//Q
uint16_t temperature_value = 0U;//Q
/****************************************************/
extern xSemaphoreHandle xBinarySemaphore_LDR      ;
extern xSemaphoreHandle xBinarySemaphore_obstacle ;

extern xQueueHandle xQueue_LDR_Difference                   ;
extern xQueueHandle xQueue_LDR_Difference_LCD               ;
extern xQueueHandle xQueue_LDR_Difference_temperature_value ;

extern uint32_t seconds_count;

extern carSide_t LEFT_SIDE;
extern carSide_t RIGHT_SIDE;


button_t start_switch = {
                         .btnPort =PORTF_ID,
                         .btnPin = PIN0_ID,
                         .btn_conn = PULL_UP,
                         .btn_state = RELEASED
};

button_t stop_switch = {
                        .btnPort =PORTF_ID,
                        .btnPin = PIN4_ID,
                        .btn_conn = PULL_UP,
                        .btn_state = RELEASED
};

LDR_Sensor_t LDR1 = {
                     .ldrPort = PORTE_ID,
                     .ldrPin = PIN1_ID,
                     .adcChannel = CH_1,
                     .sequencer = SS_0,
                     .sample = S_0,
                     .adc_module = ADC_0,
                     .temp_sensor_mode =NO

};

LDR_Sensor_t LDR2 = {
                     .ldrPort = PORTE_ID,
                     .ldrPin = PIN2_ID,
                     .adcChannel = CH_2,
                     .sequencer = SS_0,
                     .sample = S_0,
                     .adc_module = ADC_1,
                     .temp_sensor_mode =NO
};

Temp_Sensor_t TEMP_SENSOR = {
                             .sensorPort = PORTE_ID,
                             .sensorPin = PIN3_ID,
                             .adcChannel = CH_0,
                             .sequencer = SS_3,
                             .sample = S_0,
                             .adc_module = ADC_1,
                             .temp_sensor_mode =YES

};

ultraSonic_t Ultrasonic_Sensor = {
                                  .ultrasonicPort = PORTC_ID,
                                  .ultrasonicTriggerPin = PIN5_ID,
                                  .ultrasonicEchoPin = PIN6_ID
};

GPTM_ConfigType Stop_watch = {
                              .timer_global_operation = _16BIT_IN_16_OR_32_BIT_TIMER,
                              .timer_mode = PERIODIC,
                              .timer_number = TIMER3,
                              .timer_type = _16_OR_32_BIT_TIMER,
                              .timer_use = INDIVIDUAL_A
};

void manage_time(void)
{
    static uint32_t secs ;
    secs = seconds_count% 60 ;
    lcd_move_cursor(1,5);
    if(secs >= 10)
    {
        lcd_move_cursor(1,6);
        lcd_intger_to_string(secs);
    }else if (secs == 0)
    {
        lcd_move_cursor(1,6);
        lcd_display_string("00");
    }
    else {
        lcd_move_cursor(1,7);
        lcd_intger_to_string(secs);
    }
}

void stopwatch_init(void)
{

    timer_init(&Stop_watch, 1000);
}

void system_init(void)
{
    /*initialization of lcd */
    lcd_init();

    /*car side configuration initialization*/
    car_init(&LEFT_SIDE , &RIGHT_SIDE);

    /*LDR init*/
    ldr_sensor_init(&LDR1);
    ldr_sensor_init(&LDR2);

    /*TEMP_Sensor init*/
    temperature_sensor_init(&TEMP_SENSOR);

    /*Ultrasonic init*/
    ultarsonic_sensor_init(&Ultrasonic_Sensor);

    /*start switch init*/
    button_init(&start_switch);

    /*stop switch init*/
    button_init(&stop_switch);
    stopwatch_init();
}
void switches_update(void *ptr)
{
    static buttonState_t current_start_switch_state , last_start_switch_state = RELEASED;
    static buttonState_t current_stop_switch_state , last_stop_switch_state = RELEASED;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        button_get_state(&start_switch, &current_start_switch_state);
        button_get_state(&stop_switch, &current_stop_switch_state);

        /****************** Start operation************************/
        /*when both current and last are equal do nothing*/
        if(current_start_switch_state == PRESSED && last_start_switch_state == RELEASED)
        {
            robot_move_stop();
            vTaskResume(TaskHandle_ultrasonic) ;
            vTaskResume(TaskHandle_ldr       ) ;
            vTaskResume(TaskHandle_robot     ) ;
            vTaskResume(TaskHandle_lcd       ) ;
            vTaskResume(TaskHandle_temp      ) ;
            /*reset second counter*/
            seconds_count = 0;
        }
        else /*Button_currentState == RELEASED && Button_lastState == PRESSED*/
        {
            /*then we are in the pre-release state */
            /*make the last stated = pressed*/
            last_start_switch_state = current_start_switch_state;
        }
        /****************** Stop operation************************/
        if(seconds_count >= 60 || ( current_stop_switch_state == PRESSED && last_stop_switch_state == RELEASED))
        {
            vTaskSuspend(TaskHandle_ultrasonic) ;
            vTaskSuspend(TaskHandle_ldr       ) ;
            vTaskSuspend(TaskHandle_robot     ) ;
            vTaskSuspend(TaskHandle_lcd       ) ;
            vTaskSuspend(TaskHandle_temp      ) ;
            robot_move_stop();
        }
        else /*Button_currentState == RELEASED && Button_lastState == PRESSED*/
        {
            /*then we are in the pre-release state */
            /*make the last stated = pressed*/
            last_stop_switch_state = current_stop_switch_state;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
void lcd_update(void *ptr)
{
    int32_t temperature_value ;
    int32_t LDR_Difference_LCD ;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /*Set cursor position to ROW_0 and COL_0*/
        lcd_move_cursor(0,0);
        /*Displaying Temperature sensor value*/
        lcd_display_string("Temp = ");
        xQueueReceive(xQueue_LDR_Difference_temperature_value , &temperature_value, 0 );
        lcd_intger_to_string(temperature_value);//Q
        lcd_display_string(" C");
        /*Displaying Time */

        manage_time();
        /*clear this part old LCD*/
        lcd_move_cursor(1,10);
        lcd_display_string("     ");
        /*Displaying LDR difference value*/
        lcd_move_cursor(1,10);
        xQueueReceive(xQueue_LDR_Difference_LCD , &LDR_Difference_LCD, 0 );
        lcd_intger_to_string(LDR_Difference_LCD);//Q
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
    }
}
void ultrasonic_update(void *ptr)
{
    uint16_t obstacle_distance = 0U;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /*get distance*/
        obstacle_distance = ultarsonic_sensor_read_value(&Ultrasonic_Sensor);
        /*check the car is blocked*/
        if((10U < obstacle_distance) || (0U == obstacle_distance ))
        {
           /*do nothing*/
        }
        else
        {
            xSemaphoreGive(xBinarySemaphore_obstacle) ;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
void ldr_update(void *ptr)
{
    int32_t LDR_Difference = 0;
    int32_t LDR_Difference_LCD = 0;
    uint16_t LDR1_Read = 0U;
    uint16_t LDR2_Read = 0U;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        LDR1_Read = ldr_sensor_reading(&LDR1);
        LDR2_Read = ldr_sensor_reading(&LDR2);

        LDR_Difference = LDR1_Read - LDR2_Read;
        ptr = & LDR_Difference ;
        xQueueSendToBack(xQueue_LDR_Difference, ptr, 5);

        LDR_Difference_LCD = LDR_Difference;
        ptr = & LDR_Difference_LCD ;
        xQueueSendToBack(xQueue_LDR_Difference_LCD, ptr, 5);

        /*check LDRs value*/
        if( (LDR_SET_POINT_POSTIVE > LDR_Difference) && (LDR_Difference > 0) )
        {
            /*do nothing*/
        }
        else
        {
            xSemaphoreGive(xBinarySemaphore_LDR) ;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void robot_state(void *ptr)
{
    int32_t LDR_Difference ;
    uint8_t CAR_90FLAG=0U;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        if(xSemaphoreTake(xBinarySemaphore_obstacle , 1))
        {
            CAR_90FLAG=1U;

            robot_move_back();
        }
        else
        {
            if(CAR_90FLAG==1U )
            {
                CAR_90FLAG=0U;
                robot_move_rotate();
                /*delay to rotate 90*/
                vTaskDelay(pdMS_TO_TICKS (700)) ;
            }
            else
            {
                robot_move_forward();
                if(xSemaphoreTake(xBinarySemaphore_LDR , 1))
                {
                    xQueueReceive( xQueue_LDR_Difference , &LDR_Difference, 0 );
                    if(LDR_Difference > 0 )
                    {
                        robot_move_right();
                    }
                    else if(LDR_Difference < 0)
                    {
                        robot_move_left();
                    }
                    else
                    {
                        /*Do nothing*/
                    }
                }
                else
                {
                    /*Do nothing*/
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
void temperature_update(void *ptr)
{
    int32_t temperature_value ;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        temperature_value = temperature_sensor_reading(&TEMP_SENSOR);
        temperature_value = (147.5 - ((75*3.3*temperature_value)/4096));
        ptr = & temperature_value ;
        xQueueSendToBack(xQueue_LDR_Difference_temperature_value, ptr, 5);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}
