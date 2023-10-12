/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   general_functions.c                                          *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for General Functions driver                     *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "general_functions.h"

/*
 * Description:
 * This function provides a blocking delay in milliseconds.
 * It loops for the specified number of milliseconds, causing
 * a time delay.
 * Parameters:
 * - delay: The number of milliseconds to delay.
 * Returns: None.
 */
void _delay_ms(uint32_t delay)
{
    uint32_t i, j;

    /* Loop for the specified number of milliseconds */
    for (i = 0; i < delay; i++)
    {
        /* Nested loop to create a delay of approximately 1 millisecond */
        for (j = 0; j < 3180; j++)
        {
            /* Do nothing, just waste time */
        }
    }
}

/*
 * Description:
 * This function provides a blocking delay in microseconds.
 * It loops for the specified number of microseconds, causing
 * a time delay.
 * Parameters:
 * - delay: The number of microseconds to delay.
 * Returns: None.
 */
void _delay_us(uint32_t delay)
{
    uint32_t i, j;

    /* Loop for the specified number of microseconds */
    for (i = 0; i < delay; i++)
    {
        /* Nested loop to create a delay of approximately 1 microseconds */
        for (j = 0; j < 4; j++)
        {
            /* Do nothing, just waste time */
        }
    }
}


