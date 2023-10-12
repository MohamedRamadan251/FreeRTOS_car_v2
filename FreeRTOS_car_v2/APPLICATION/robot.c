/*
 * robot.c
 *
 *  Created on: Sep 23, 2023
 *      Author: Kareem Abdelrasheed
 */

#include "robot.h"

carSide_t LEFT_SIDE  = {
                        .carSidePort = PORTB_ID,
                        .carSideDir1Pin = PIN7_ID,
                        .carSideDir2Pin = PIN1_ID,
                        .carSideEnPin = PIN6_ID,
                        .carSideSpeed = 30

};

carSide_t RIGHT_SIDE = {
                        .carSidePort = PORTB_ID,
                        .carSideDir1Pin = PIN4_ID,
                        .carSideDir2Pin = PIN5_ID,
                        .carSideEnPin = PIN0_ID,
                        .carSideSpeed = 30
};

void robot_move_back(void)
{
    car_side_direction_set(&LEFT_SIDE , forward);
    car_side_direction_set(&RIGHT_SIDE , forward);
}
void robot_move_forward(void)
{
    car_side_direction_set(&LEFT_SIDE , backward);
    car_side_direction_set(&RIGHT_SIDE , backward);
}
void robot_move_stop(void)
{
    car_side_direction_set(&LEFT_SIDE , stop);
    car_side_direction_set(&RIGHT_SIDE , stop);
}
void robot_move_left(void)
{
    car_side_direction_set(&LEFT_SIDE , stop);
    car_side_direction_set(&RIGHT_SIDE , backward);

}
void robot_move_right(void)
{
    car_side_direction_set(&LEFT_SIDE , backward);
    car_side_direction_set(&RIGHT_SIDE , stop);
}
void robot_move_rotate(void)
{
    car_side_direction_set(&LEFT_SIDE , backward);
    car_side_direction_set(&RIGHT_SIDE , forward);
}


