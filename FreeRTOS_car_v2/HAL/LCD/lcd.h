/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   lcd.h                                                        *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: header file for LCD driver                                   *
 *                                                                             *
 *******************************************************************************/

#ifndef LCD_H_
#define LCD_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* LCD Data bits mode configuration, its value should be 4 or 8 */
#define LCD_DATA_BITS_MODE 4

#if((LCD_DATA_BITS_MODE != 4) && (LCD_DATA_BITS_MODE != 8))

#error "Number of Data bits should be equal to 4 or 8"

#endif

/* LCD HW Ports and Pins IDs */
#define LCD_RS_PORT_ID                 PORTF_ID
#define LCD_RS_PIN_ID                  PIN1_ID

#define LCD_E_PORT_ID                  PORTD_ID
#define LCD_E_PIN_ID                   PIN2_ID

#define LCD_DATA_PORT_ID               PORTA_ID

#if (LCD_DATA_BITS_MODE == 4)

#define LCD_DB4_PIN_ID                 PIN2_ID
#define LCD_DB5_PIN_ID                 PIN3_ID
#define LCD_DB6_PIN_ID                 PIN4_ID
#define LCD_DB7_PIN_ID                 PIN5_ID

#endif

/* LCD Commands */
#define LCD_CLEAR_COMMAND                    0x01
#define LCD_GO_TO_HOME                       0x02
#define LCD_TWO_LINES_EIGHT_BITS_MODE        0x38
#define LCD_TWO_LINES_FOUR_BITS_MODE         0x28
#define LCD_TWO_LINES_FOUR_BITS_MODE_INIT1   0x33
#define LCD_TWO_LINES_FOUR_BITS_MODE_INIT2   0x32
#define LCD_CURSOR_OFF                       0x0C
#define LCD_CURSOR_ON                        0x0E
#define LCD_SET_CURSOR_LOCATION              0x80

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Description :
 * Initialize the LCD:
 * 1. Setup the LCD pins directions by use the DIO driver.
 * 2. Setup the LCD Data Mode 4-bits or 8-bits.
 * Inputs: None.
 * Returns: None.
 */
void lcd_init(void);

/*
 * Description :
 * Send the required command to the screen.
 * Parameters:
 * - command -> The command to be sent to the LCD.
 * Returns: None.
 */
void lcd_send_command(uint8_t command);

/*
 * Description :
 * Display the required character on the screen
 * Parameters:
 * - data -> The character data to be displayed on the LCD.
 * Returns: None.
 */
void lcd_display_character(uint8_t data);

/*
 * Description :
 * Display the required string on the screen.
 * Parameters:
 * - Str -> The null-terminated string to be displayed on the LCD.
 * Returns: None.
 */
void lcd_display_string(const char *Str);

/*
 * Description :
 * Move the cursor to a specified row and column index on the screen.
 * Parameters:
 * - row: The row (0 to 3) where you want to move the cursor.
 * - col: The column (0 to 15) where you want to move the cursor.
 * Returns: None.
 */
void lcd_move_cursor(uint8_t row,uint8_t col);

/*
 * Description :
 * Display the required string in a specified row and column index on the screen.
 * Parameters:
 * - row: The row (0 to 3) where you want to display the string.
 * - col: The column (0 to 15) where you want to display the string.
 * - Str: The null-terminated string to be displayed.
 * Returns: None.
 */
void lcd_display_string_row_column(uint8_t row,uint8_t col,const char *Str);

/*
 * Description :
 * Display the required decimal value on the screen.
 * Parameters:
 * - data: The integer value to be converted and displayed.
 * Returns: None.
 */
void lcd_intger_to_string(sint32_t data);

/*
 * Description :
 * Send the clear screen command.
 * Parameters: None.
 * Returns: None.
 */
void lcd_clear_screen(void);

#endif /* LCD_H_ */
