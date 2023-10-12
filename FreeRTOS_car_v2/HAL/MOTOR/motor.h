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
 * [DESCRIPTION]: header file for motor driver                                 *
 *                                                                             *
 *******************************************************************************/

#ifndef HAL_MOTOR_MOTOR_H_
#define HAL_MOTOR_MOTOR_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/*
 * Description:
 * Struct to specify configuration of each motor in a car side,
 * including GPIO and TIMER.
 */
typedef struct
{
    uint8_t  carSidePort;
    uint8_t  carSideDir1Pin;
    uint8_t  carSideDir2Pin;
    uint8_t  carSideEnPin;
    uint8_t  carSideSpeed;
}carSide_t;

/*
 * Description:
 * Struct to specify direction of each motor in a car side,
 * including GPIO.
 */
typedef enum
{
    forward=0,
    backward,
    stop
}carSideDirection_t;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * Description:
 * This function initializes a car side pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors or PWM signal to control speed.
 * Parameters:
 * - carSide: pointer to the car side configuration (PORT_ID , PIN_ID , PWM_PIN , INITIAL_SPEED).
 * Returns: None.
 */

void car_init(carSide_t * sideA , carSide_t * sideB);

/*
 * Description:
 * This function set a car side direction configuration.
 * It configures the direction of each side to be forward or backward or to stop the side,
 * Parameters:
 * - carSide: pointer to the car side configuration (PORT_ID , PIN_ID , PWM_PIN , INITIAL_SPEED).
 * - direction: The desired car side direction (forward , backward, stop).
 * Returns: None.
 */
void car_side_direction_set(carSide_t * carSide , carSideDirection_t direction);


#endif /* HAL_MOTOR_MOTOR_H_ */
