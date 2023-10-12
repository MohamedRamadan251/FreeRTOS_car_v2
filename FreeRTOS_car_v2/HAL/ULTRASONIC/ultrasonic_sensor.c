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
 * [DESCRIPTION]: source file for ultrasonic sensor driver                     *
 *                                                                             *
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "tm4c123gh6pm.h"

#include "LIB/bitwise_operation.h"


#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/TIMER/timer.h"
#include "../../MCAL/general_functions.h"

#include "ultrasonic_sensor.h"

void ultarsonic_sensor_init(ultraSonic_t *sensor)
{       /*Set echo pin as input*/
        gpio_init(sensor->ultrasonicPort, sensor->ultrasonicEchoPin , PIN_INPUT_PULL_DOWN);
        /*Set trigger pin as output*/
        gpio_init(sensor->ultrasonicPort,  sensor->ultrasonicTriggerPin , PIN_OUTPUT_PULL_DOWN);

        /*give clock to timer*/
        SYSCTL_RCGCTIMER_R |= 1U<<1;

        /*Disable Timer1-A to use it to calculate time*/
        CLEAR_BIT(TIMER1_CTL_R , 0);
        /*Configure Timer1-A to one shot mode in  down counting mode*/
        TIMER1_CFG_R |= 0x04;
        TIMER1_TAMR_R |= 0x01;
        /*To stop timer during debugging*/
        SET_BIT(TIMER1_CTL_R , 1);
        /*To load timer with initial value = 0xFFFF */
        TIMER1_TAILR_R = 0xFFFF;

}

uint32_t ultarsonic_sensor_read_value(ultraSonic_t *sensor)
{
        /*Variable to store time of ultrasonic*/
        uint32_t duration = 0U;
        /*Variable to store distance*/
        uint32_t reading = 0U ;
        /*Variable to store the state of echo pin*/
        uint32_t pinValue = 0U ;
        /*Sending 10uS pulse on trigger pin to enable ultrasonic sensor*/
        gpio_write_pin(sensor->ultrasonicPort, sensor->ultrasonicTriggerPin, 1);
        /*Delay 10 micro Second*/
        _delay_us(10U);
        gpio_write_pin(sensor->ultrasonicPort, sensor->ultrasonicTriggerPin, 0);

        pinValue =  gpio_read_pin(sensor->ultrasonicPort, sensor->ultrasonicEchoPin);
        /*Waiting for echo pin to be high*/
        while(pinValue == 0)
        {
            pinValue =  gpio_read_pin(sensor->ultrasonicPort, sensor->ultrasonicEchoPin);
        }
        /*Start calculating time*/
        TIMER1_TAILR_R = 0xFFFF;
        SET_BIT(TIMER1_CTL_R , 0);

        /*Waiting for echo pin to be low*/
        pinValue =  gpio_read_pin(sensor->ultrasonicPort, sensor->ultrasonicEchoPin);
        while(pinValue == 1)
        {
            pinValue =  gpio_read_pin(sensor->ultrasonicPort, sensor->ultrasonicEchoPin);
        }
        /*Getting the time value*/
        duration = TIMER1_TAR_R;
        /*Calculate the actual time*/
        duration = 0xffff - duration;
        /*Disable timer1-A*/
        CLEAR_BIT(TIMER1_CTL_R , 0);
        /*Calculating distance in cm*/
        reading = (duration *340.0 *100) / (2.0*16000000);
        /*Returning distance*/
        return reading;
}

