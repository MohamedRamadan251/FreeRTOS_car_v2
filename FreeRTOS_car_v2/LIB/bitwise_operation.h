/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   bitwise_operator.h                                           *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: header file for the bitwise operations                       *
 *                                                                             *
 *******************************************************************************/

#ifndef BITWISE_OPERATION
#define BITWISE_OPERATION

/* Set a certain bit in any register */
#define SET_BIT(REG,BIT) (REG|=(1U<<BIT))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG,BIT) (REG&=(~(1U<<BIT)))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG,BIT) (REG^=(1U<<BIT))

/* Modify a certain bit in any register */
#define MODIFY_BIT(REG, BIT, VALUE) ((VALUE) ? ((REG) |= (1U << (BIT))) : ((REG) &= ~(1U << (BIT))))

/* Rotate right the register value with specific number of rotates */
#define ROR(REG,num) ( REG= (REG>>num) | (REG<<(8U-num)) )

/* Rotate left the register value with specific number of rotates */
#define ROL(REG,num) ( REG= (REG<<num) | (REG>>(8U-num)) )

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG,BIT) ( REG & (1U<<BIT) )

/* Check if a specific bit is cleared in any register and return true if yes */
#define BIT_IS_CLEAR(REG,BIT) ( !(REG & (1U<<BIT)) )

/* Get the value of a certain bit in any register */
#define GET_BIT(REG,BIT) ( ( REG & (1U<<BIT) ) >> BIT )

#endif
