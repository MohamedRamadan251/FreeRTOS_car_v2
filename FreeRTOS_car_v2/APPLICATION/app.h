/*
 * APP.h
 *
 *  Created on: Sep 13, 2023
 *      Author: Kareem Abdelrasheed
 */

#ifndef APPLICATION_APP_H_
#define APPLICATION_APP_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "../LIB/bitwise_operation.h"
#include "../LIB/std_types.h"

#include "../HAL/LCD/lcd.h"
#include "../HAL/LDR/ldr_sensor.h"
#include "../HAL/MOTOR/motor.h"
#include "../HAL/SWITCH/switch.h"
#include "../HAL/ULTRASONIC/ultrasonic_sensor.h"
#include "../HAL/TEMP_SENSOR/temperture_sensor.h"

#include "../MCAL/GPIO/gpio.h"
#include "../MCAL/TIMER/timer.h"
#include "../MCAL/general_functions.h"

#include "robot.h"

void system_init(void);

/*Program tasks*/
void switches_update(void *ptr);
void lcd_update(void *ptr);
void ultrasonic_update(void *ptr);
void ldr_update(void *ptr);
void temperature_update(void *ptr);
void robot_state(void *ptr);
void manage_time(void);

#endif /* APPLICATION_APP_H_ */
