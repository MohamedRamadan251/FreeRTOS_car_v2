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
 * [DESCRIPTION]: header file for ultrasonic sensor driver                     *
 *                                                                             *
 *******************************************************************************/

#ifndef HAL_ULTRASONIC_SENSOR_ULTRASONIC_SENSOR_H_
#define HAL_ULTRASONIC_SENSOR_ULTRASONIC_SENSOR_H_

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
 * Struct to specify configuration ultrasonic sensor,
 * including GPIO and TIMER.
 */
typedef struct {
    uint8_t ultrasonicPort;
    uint8_t ultrasonicEchoPin;
    uint8_t ultrasonicTriggerPin;
}ultraSonic_t;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description:
 * This function initializes the ultrasonic sensor pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the ultrasonic sensor configuration (PORT_ID , ECHO_PIN , TRIGGER_PIN).
 * Returns: None.
 */
void ultarsonic_sensor_init(ultraSonic_t *sensor);

/*
 * Description:
 * This function gets the ultrasonic sensor reading (distance from obstacle).
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors or PWM signal to control speed.
 * Parameters:
 * - carSide: pointer to the ultrasonic sensor configuration (PORT_ID , ECHO_PIN , TRIGGER_PIN).
 * Returns: None.
 */
uint32_t ultarsonic_sensor_read_value(ultraSonic_t *sensor);

#endif /* HAL_ULTRASONIC_SENSOR_ULTRASONIC_SENSOR_H_ */
