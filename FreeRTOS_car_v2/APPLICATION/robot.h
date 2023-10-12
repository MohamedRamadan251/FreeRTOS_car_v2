/*
 * robot.h
 *
 *  Created on: Sep 23, 2023
 *      Author: Kareem Abdelrasheed
 */

#ifndef APPLICATION_ROBOT_H_
#define APPLICATION_ROBOT_H_


#include "../HAL/MOTOR/motor.h"
#include "../MCAL/GPIO/gpio.h"
#include "../MCAL/TIMER/timer.h"
#include "../MCAL/general_functions.h"


/*Robot motion functions*/
void robot_move_forward(void);
void robot_move_back(void);
void robot_move_stop(void);
void robot_move_left(void);
void robot_move_right(void);
void robot_move_rotate(void);


#endif /* APPLICATION_ROBOT_H_ */
