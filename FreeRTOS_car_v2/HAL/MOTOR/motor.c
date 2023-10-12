/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   gpio.h                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for motor driver                                  *
 *                                                                             *
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "LIB/bitwise_operation.h"


#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/TIMER/timer.h"
#include "../../MCAL/general_functions.h"

#include "motor.h"
void car_init(carSide_t * sideA , carSide_t * sideB)
{
    GPTM_ConfigType side_a = {
                             .timer_number = TIMER0,
                             .timer_mode = PERIODIC,
                             .timer_global_operation = _16BIT_IN_16_OR_32_BIT_TIMER,
                             .timer_type = _16_OR_32_BIT_TIMER,
                             .timer_use =  INDIVIDUAL_A
    };

    GPTM_ConfigType side_b = {
                             .timer_number = TIMER2,
                             .timer_mode = PERIODIC,
                             .timer_global_operation = _16BIT_IN_16_OR_32_BIT_TIMER,
                             .timer_type = _16_OR_32_BIT_TIMER ,
                             .timer_use = INDIVIDUAL_A
    };
    /*pins direction*/
    /*side A*/
    gpio_init(sideA->carSidePort , sideA->carSideDir1Pin , PIN_OUTPUT_PULL_DOWN);
    gpio_init(sideA->carSidePort , sideA->carSideDir2Pin , PIN_OUTPUT_PULL_DOWN);
    /*alternative pin for PWM (timer)*/
    gpio_alternative_function(sideA->carSidePort , sideA->carSideEnPin , PWM);
    pwm_init(&side_a);

    /*side B*/
    gpio_init(sideB->carSidePort , sideB->carSideDir1Pin , PIN_OUTPUT_PULL_DOWN);
    gpio_init(sideB->carSidePort , sideB->carSideDir2Pin , PIN_OUTPUT_PULL_DOWN);
    /*alternative pin for PWM (timer) */
    gpio_alternative_function(sideB->carSidePort , sideB->carSideEnPin , PWM);
    pwm_init(&side_b);


    pwm_start(&side_a, ( sideA->carSideSpeed) * 0xFFFF / 100);

    pwm_start(&side_b, ( sideB->carSideSpeed) * 0xFFFF / 100);

}

void car_side_direction_set(carSide_t * carSide , carSideDirection_t direction)
{

    switch(direction)
    {
    case forward:
        /*Set direction to moving forward*/
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir1Pin , 1 );
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir2Pin , 0 );
        break;
    case backward:
        /*Set direction to moving backward*/
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir1Pin , 0 );
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir2Pin , 1 );
        break;
    case stop:
        /*Stop car side motors*/
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir1Pin , 0 );
        gpio_write_pin(carSide->carSidePort , carSide->carSideDir2Pin , 0 );
        break;
    default:
        break;
    }

}


