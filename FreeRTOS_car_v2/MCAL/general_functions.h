/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   general_functions.h                                          *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: header file for General Functions driver                     *
 *                                                                             *
 *******************************************************************************/

#ifndef GENERAL_FUNCTIONS_H_
#define GENERAL_FUNCTIONS_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * Description:
 * This function provides a blocking delay in milliseconds.
 * It loops for the specified number of milliseconds, causing
 * a time delay.
 * Parameters:
 * - delay: The number of milliseconds to delay.
 * Returns: None.
 */
void _delay_ms(uint32_t delay);
/*
 * Description:
 * This function provides a blocking delay in microseconds.
 * It loops for the specified number of microseconds, causing
 * a time delay.
 * Parameters:
 * - delay: The number of microseconds to delay.
 * Returns: None.
 */

void _delay_us(uint32_t delay);
#endif /* GENERAL_FUNCTIONS_H_ */
