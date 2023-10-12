/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   std_types.h                                                  *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: header file for the standard types                           *
 *                                                                             *
 *******************************************************************************/

#ifndef STD_TYPES_H_
#define STD_TYPES_H_
#include <stdbool.h>
#include <stdint.h>

/* Boolean Values */
#ifndef FALSE
#define FALSE       (0u)
#endif
#ifndef TRUE
#define TRUE        (1u)
#endif

#define LOGIC_HIGH        (1u)
#define LOGIC_LOW         (0u)



typedef signed char           sint8_t;          /*        -128 .. +127             */
typedef signed short          sint16_t;         /*      -32768 .. +32767           */
typedef signed long           sint32_t;         /* -2147483648 .. +2147483647      */
typedef signed long long      sint64_t;         /* -9223372036854775808 .. 9223372036854775807 */
typedef float                 float32_t;
typedef double                float64_t;

#endif /* STD_TYPE_H_ */
